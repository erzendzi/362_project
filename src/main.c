/**
  ******************************************************************************
  * @file    main.c
  * @author  Erek Rzendzian
  * @date    Feb 7, 2024
  * @brief   ECE 362 Project
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdint.h>
#include "commands.h"
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
#include "parse_bmp.h"
#include "ff.h"
#include "lcd.h"
#include <stdlib.h> // For malloc and free
#include <math.h>   // for M_PI

// FATFS Object
FATFS fs_storage;



// USART5 FIFO Buffer Definitions (Assuming existing)
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void internal_clock();

void init_usart5() {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    // Map PC12 to USART5_TX
    GPIOC->MODER &= ~(0x3 << (12 * 2)); // PC12 clear
    GPIOC->MODER |= (0x2 << (12 * 2)); // PC12 AF
    GPIOC->AFR[1] &= ~(0xF << ((12 - 8) * 4)); // PC12 AF clear
    GPIOC->AFR[1] |= (0x2 << ((12 - 8) * 4)); // PC12 AF2

    // Map PD2 to USART5_RX
    GPIOD->MODER &= ~(0x3 << (2 * 2)); // PD2 clear
    GPIOD->MODER |= (0x2 << (2 * 2)); // PD2 AF
    GPIOD->AFR[0] &= ~(0xF << (2 * 4)); // PD2 AF clear
    GPIOD->AFR[0] |= (0x2 << (2 * 4)); // PD2 AF2

    
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN; // Enable the RCC clock to the USART5 peripheral

    USART5->CR1 &= ~USART_CR1_UE; // Disable USART 5

    USART5->CR1 &= ~USART_CR1_M; // Set word length to 8 bits
    USART5->CR2 &= ~USART_CR2_STOP; // Set for 1 stop bit
    USART5->CR1 &= ~USART_CR1_PCE; // Disable partity
    USART5->CR1 &= ~USART_CR1_OVER8; // Set 16x oversampling
    USART5->BRR = 0x1A1; // 115.2kbaud
    USART5->CR1 |= USART_CR1_TE; // TE 
    USART5->CR1 |= USART_CR1_RE; // RE

    USART5->CR1 |= USART_CR1_UE; // Reenable USART5

    while (!(USART5->ISR & USART_ISR_TEACK) || !(USART5->ISR & USART_ISR_REACK)) {} // Wait for acknowledgement of TE and RE
}

void enable_tty_interrupt(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;

    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);
    DMA2_Channel2->CNDTR = FIFOSIZE;

    // Config for DMA2 (slimmed it down by combining)
    DMA2_Channel2->CCR &= ~(DMA_CCR_DIR | DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_MSIZE | DMA_CCR_PSIZE);
    DMA2_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL;
    DMA2_Channel2->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_MEM2MEM);

    DMA2_Channel2->CCR |= DMA_CCR_EN;

    USART5->CR3 |= USART_CR3_DMAR;
    USART5->CR1 |= USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART3_8_IRQn);
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    while (!fifo_newline(&input_fifo)) {
        asm volatile ("wfi");  // wait for an interrupt
    }

    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_putchar(int c) {
    if(c == 10) {
        while(!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = 13;
    }

    while(!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    return interrupt_getchar();
}

void USART3_8_IRQHandler(void) {
    while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
        if (!fifo_full(&input_fifo))
            insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

void init_spi1() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PB3, PB5 to Alternate Function (AF0)
    GPIOB->MODER &= ~((0x3 << (3*2)) | (0x3 << (5*2)));
    GPIOB->MODER |= 0x2 << (3*2) | 0x2 << (5*2);
    GPIOB->AFR[0] &= ~((0xF << (3 * 4)) | (0xF << (5 * 4)));

    // Disable SPI1 before configuration
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Configure SPI1

    SPI1->CR1 |= SPI_CR1_MSTR;       // Master mode
    SPI1->CR1 |= SPI_CR1_BR_0;  // Baud rate divisor (max)
    
    SPI1->CR2 &= ~(0xF << SPI_CR2_DS_Pos);
    SPI1->CR2 |= 0x7 << SPI_CR2_DS_Pos; // 8-bit word size

    SPI1->CR1 |= SPI_CR1_SSM;        // Software slave management
    SPI1->CR1 |= SPI_CR1_SSI;       // Internal slave select

    // Set FIFO reception threshold
    SPI1->CR2 |= SPI_CR2_FRXTH;
    
    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void init_spi2_slow() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // Configure PB13, PB14, PB15 to Alternate Function (AF0)
    GPIOB->MODER &= ~(0x3F << (13*2));
    GPIOB->MODER |= 0x2A << (13*2);
    GPIOB->AFR[1] &= ~((0xFFF << (5 * 4)));

    // Disable SPI1 before configuration
    SPI2->CR1 &= ~SPI_CR1_SPE;

    // Configure SPI2
    
    SPI2->CR1 |= SPI_CR1_MSTR;       // Master mode
    SPI2->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;  // Baud rate divisor (max)
    
    SPI2->CR2 &= ~(0xF << SPI_CR2_DS_Pos);
    SPI2->CR2 |= 0x7 << SPI_CR2_DS_Pos; // 8-bit word size

    SPI2->CR1 |= SPI_CR1_SSM;        // Software slave management
    SPI2->CR1 |= SPI_CR1_SSI;       // Internal slave select

    // Set FIFO reception threshold
    SPI2->CR2 |= SPI_CR2_FRXTH;

    // Enable SPI2
    SPI2->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard() {
    // Ensure PB2 is configured as output
    GPIOB->MODER &= ~(0x3 << (2 * 2));
    GPIOB->MODER |= (0x1 << (2 * 2));

    // Set PB2 low
    GPIOB->BRR = (1 << 2); // Reset bit for PB2
}

void disable_sdcard() {
    // Set PB2 high
    GPIOB->BSRR = (1 << 2); // Set bit for PB2
}

void init_sdcard_io() {
    init_spi2_slow();

    // Configure PB2 as output
    GPIOB->MODER &= ~(0x3 << (2 * 2));
    GPIOB->MODER |= (0x1 << (2 * 2));

    // Disable SD card
    disable_sdcard();
}

void sdcard_io_high_speed() {
    // Disable SPI2
    SPI2->CR1 &= ~SPI_CR1_SPE;

    // Set Baud Rate to 12 MHz
    SPI2->CR1 &= ~SPI_CR1_BR;
    SPI2->CR1 |= SPI_CR1_BR_0; // BR[2:0] = 0b001

    // Re-enable SPI1
    SPI2->CR1 |= SPI_CR1_SPE;
}

void init_lcd_spi() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOAEN;

    // Configure PB8, PB11, PA0 as outputs
    GPIOB->MODER &= ~((3 << (8 * 2)) | (3 << (11 * 2)));
    GPIOB->MODER |= ((1 << (8 * 2)) | (1 << (11 * 2)));
    GPIOA->MODER &= ~0x3;
    GPIOA->MODER |= 0x1;

    // Initialize SPI1 for LCD
    init_spi1();
    

    init_spi2_slow();
    // Increase SPI2 speed
    sdcard_io_high_speed();

}

//===========================================================================
/**
 * @brief Configures DMA1 Channel3 for SPI1_TX to transfer pixel data.
 * 
 * This function sets up DMA1 Channel3 to read from spi_buffer and write to SPI1's data register.
 * It configures the DMA channel for memory-to-peripheral transfer with incrementing memory address,
 * 8-bit data size, and no circular mode.
 * 
 * @param buffer Pointer to the data buffer to be transmitted via SPI1.
 * @param length Number of bytes to transfer.
 */
//===========================================================================
void spi1_setup_dma(uint8_t *buffer, uint16_t length) {
    // Enable DMA1 clock
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    
    // Disable DMA Channel3 before configuring
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    
    // Configure DMA Channel3
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR; // Peripheral address: SPI1 Data Register
    DMA1_Channel3->CMAR = (uint32_t)buffer;    // Memory address: buffer to send
    DMA1_Channel3->CNDTR = length;            // Number of data items to transfer
    
    // Configure DMA Channel3
    DMA1_Channel3->CCR &= ~(DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_PSIZE | DMA_CCR_MSIZE);
    DMA1_Channel3->CCR |= DMA_CCR_MINC;       // Memory increment mode
    DMA1_Channel3->CCR |= DMA_CCR_DIR;      // Read from memory (DIR=1)
    DMA1_Channel3->CCR |= DMA_CCR_TCIE;       // Transfer complete interrupt (optional)
    
    // Set data size to 8 bits
    // MSIZE and PSIZE are already cleared to 00 for 8-bit
    
    // Optionally, set priority
    DMA1_Channel3->CCR |= DMA_CCR_PL_1; // High priority
}

//===========================================================================
/**
 * @brief Enables DMA1 Channel3 to start the transfer.
 */
//===========================================================================
void spi1_enable_dma(void) {
    // Enable SPI1_TX DMA requests
    SPI1->CR2 |= SPI_CR2_TXDMAEN;
    
    // Enable DMA Channel3
    DMA1_Channel3->CCR |= DMA_CCR_EN;
}

//===========================================================================
/**
 * @brief DMA1 Channel3 Transfer Complete Interrupt Handler.
 * 
 * Optional: Handle the end of DMA transfer, e.g., to toggle a GPIO pin or notify the main program.
 */
//===========================================================================
void DMA1_Channel3_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF3) {
        // Clear the transfer complete flag
        DMA1->IFCR |= DMA_IFCR_CTCIF3;
        
        // Optionally, disable DMA and SPI1 TX DMA request if not in circular mode
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;
        SPI1->CR2 &= ~SPI_CR2_TXDMAEN;
        
        // Add any post-transfer handling here (e.g., notify main loop)
    }
}

void mount_sd_card() {
    FATFS *fs = &fs_storage;
    if (fs->id != 0) {
        print_error(FR_DISK_ERR, "Already mounted.");
        return;
    }
    int res = f_mount(fs, "", 1);
    if (res != FR_OK)
        print_error(res, "Error occurred while mounting");
}

void enable_ports_audio(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~(0xFFFF); // PC0-PC7 reset
    GPIOC->MODER |= (0x55 << (4 * 2)); // PC4-PC7 outputs

    GPIOC->OTYPER |= (0xF << 4); // Config open drain PC4-PC7
    
    GPIOC->PUPDR &= ~(0xFF); // PC0-PC3 pupd reset
    GPIOC->PUPDR |= (0x55); // PC0-PC3 pu
}

//=============================================================================
// Part 2: Debounced keypad scanning.
//=============================================================================

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
// Write the Timer 7 ISR here.  Be sure to give it the right name.
void TIM7_IRQHandler() {
    TIM7->SR &= ~TIM_SR_UIF;

    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    TIM7->PSC = 47; // prescale to 1 MHz
    TIM7->ARR = 999; // reload set to 1 kHz

    TIM7->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
}

//=============================================================================
// Part 3: Analog-to-digital conversion for a volume level.
//=============================================================================
uint32_t volume = 2048;

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |= (0x3 << (2 * 1)); // Set PA1 analog
    
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->CR2 |= RCC_CR2_HSI14ON;

    while(!(RCC->CR2 & RCC_CR2_HSI14RDY)) {}

    ADC1->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
    
    ADC1->CHSELR = ADC_CHSELR_CHSEL1;

    while(!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;
    ADC1->CR |= ADC_CR_ADSTART;

    while(!(ADC1->ISR & ADC_ISR_EOC)) {}

    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE;
}


//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 47999; // Prescale down to 1 kHz
    TIM2->ARR = 99; //reload to 10 Hz

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 3);
}

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |= (0x3 << (4 * 2));

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    DAC->CR &= ~DAC_CR_TSEL1; // Sets to correct 000

    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_EN1;
}

//============================================================================
// Timer 6 ISR
//============================================================================
// Write the Timer 6 ISR here.  Be sure to give it the right name.
void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF;

    offset0 += step0;
    offset1 += step1;

    if (offset0 >= (N << 16)) {
        offset0 -= (N << 16);
    }
    if (offset1 >= (N << 16)) {
        offset1 -= (N << 16);
    }

    int samp = wavetable[offset0 >> 16] + wavetable[offset1 >> 16];
    samp *= volume;
    samp = (samp >> 17) + 2048;

    DAC->DHR12R1 = samp & 0xFFF;
}
//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    TIM6->PSC = (48000000 / (100 * RATE) - 1); // Calcualte scale based on core clock
    TIM6->ARR = 99; // Convert scale to kHz

    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= TIM_CR2_MMS_1;

    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 |= TIM_CR1_CEN;
}

// Structures to hold BMP header information
BITMAPFILEHEADER file_header;
BITMAPV5HEADER info_header;

void waveform_select(char *bmp_filename) {

    read_bmp_header(bmp_filename, &file_header, &info_header);
    display_bmp_information(bmp_filename, &file_header, &info_header);
    draw_bmp_to_lcd(bmp_filename, &file_header, &info_header);

}

char *bmp_filenames[4] = {
    "img_w_1.bmp",
    "img_w_2.bmp",
    "img_w_3.bmp",
    "img_w_4.bmp"
};

const char *loading = "Loading...";

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    enable_ports_audio();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    setup_dac();
    init_tim6();
    LCD_Setup();
    LCD_Clear(0);
    //LCD_DrawFillRectangle(0,0,200,200,0x0f0f);
    mount_sd_card();
    
    // Example: Toggle an LED to indicate success (assuming LED is connected to PA5)
    //RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOA clock
    GPIOC->MODER &= ~(0x3 << (8 * 2));  // Set PC6 as output
    GPIOC->MODER |= (0x1 << (8 * 2));  // Set PC6 as output
    GPIOC->BSRR = (1 << 8);

    // LCD is 320X240

    //char *bmp_filename = bmp_filenames[0];

    for(;;) {
        char key = get_keypress();
        if (key == 'A'){
            LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);

            waveform_select(bmp_filenames[0]);

            LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
        } else if(key == 'B'){
            LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);

            waveform_select(bmp_filenames[1]);

            LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
        } else if(key == 'C'){
            LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);

            waveform_select(bmp_filenames[2]);

            LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
        } else if(key == 'D'){
            LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);

            waveform_select(bmp_filenames[3]);

            LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
        } else {}

        //    set_freq(0,getfloat());
        //if (key == 'B')
        //    set_freq(1,getfloat());
    }
    

    // Indicate success by toggling LED
    //while(1) {
    //    GPIOC->ODR ^= (1 << 6);        // Toggle PA5
    //    for(uint32_t i = 0; i < 100000; i++); // Simple delay
    //}

    //command_shell();
}