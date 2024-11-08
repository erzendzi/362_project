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


// Define constants and arrays for multiple voices
#define NVOICES 12
int step[NVOICES] = {0};
int offset[NVOICES] = {0};

const char key_note_map[NVOICES] = {'1','2','3','4','5','6','7','8','9','*','0','#'};

const float note_freqs[NVOICES] = {
    261.63, // C4
    277.18, // C#4
    293.66, // D4
    311.13, // D#4
    329.63, // E4
    349.23, // F4
    369.99, // F#4
    392.00, // G4
    415.30, // G#4
    440.00, // A4
    466.16, // A#4
    493.88  // B4
};

//============================================================================
// ADSR Envelope Parameters and Structures
//============================================================================
#define ATTACK_TIME (RATE * 0.01)    // 10ms
#define DECAY_TIME  (RATE * 0.1)     // 100ms
#define SUSTAIN_LEVEL 0.7f           // 70% amplitude
#define RELEASE_TIME (RATE * 0.1)    // 100ms

typedef struct {
    int state;       // 0=off, 1=attack, 2=decay, 3=sustain, 4=release
    int count;       // sample count within the current state
    float amplitude; // current amplitude (0.0 to 1.0)
} ADSR;

ADSR adsr[NVOICES];

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

    int samp = 0;
    int active_voices = 0; // Count of active voices

    for (int i = 0; i < NVOICES; i++) {
        if (step[i] != 0) {
            // Update ADSR envelope
            switch(adsr[i].state) {
                case 0: // Off
                    adsr[i].amplitude = 0.0f;
                    break;
                case 1: // Attack
                    adsr[i].count++;
                    adsr[i].amplitude = (float)adsr[i].count / ATTACK_TIME;
                    if (adsr[i].amplitude >= 1.0f) {
                        adsr[i].amplitude = 1.0f;
                        adsr[i].count = 0;
                        adsr[i].state = 2; // Move to decay
                    }
                    break;
                case 2: // Decay
                    adsr[i].count++;
                    adsr[i].amplitude = 1.0f - (1.0f - SUSTAIN_LEVEL) * ((float)adsr[i].count / DECAY_TIME);
                    if (adsr[i].count >= DECAY_TIME) {
                        adsr[i].amplitude = SUSTAIN_LEVEL;
                        adsr[i].count = 0;
                        adsr[i].state = 3; // Move to sustain
                    }
                    break;
                case 3: // Sustain
                    // Maintain sustain level
                    adsr[i].amplitude = SUSTAIN_LEVEL;
                    break;
                case 4: // Release
                    adsr[i].count++;
                    adsr[i].amplitude *= 1.0f - (1.0f / RELEASE_TIME);
                    if (adsr[i].count >= RELEASE_TIME || adsr[i].amplitude <= 0.0f) {
                        // Note is off
                        adsr[i].state = 0;
                        adsr[i].amplitude = 0.0f;
                        step[i] = 0;
                        offset[i] = 0;
                        continue; // Skip to next voice
                    }
                    break;
            }

            if (adsr[i].state != 0) {
                offset[i] += step[i];
                if (offset[i] >= (N << 16)) {
                    offset[i] -= (N << 16);
                }
                samp += (int)(adsr[i].amplitude * wavetable[offset[i] >> 16]);
                active_voices++; // Increment active voices count
            }
        }
    }

    if (active_voices > 1) {
        samp = samp / active_voices; // Average the sample
    }

    samp = samp * volume;
    samp = (samp >> 17) + 2048;

    if (samp > 4095)
        samp = 4095;
    else if (samp < 0)
        samp = 0;

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

//============================================================================
// Function to Load Waveform from SD Card
//============================================================================
int load_waveform_from_sdcard(const char *filename) {
    FIL file;
    UINT bytes_read;
    FRESULT res;

    res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("Failed to open waveform file %s\n", filename);
        return -1;
    }

    res = f_read(&file, wavetable, N * sizeof(short int), &bytes_read);
    if (res != FR_OK || bytes_read != N * sizeof(short int)) {
        printf("Failed to read waveform data from %s\n", filename);
        f_close(&file);
        return -1;
    }

    f_close(&file);
    return 0;
}

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
    //init_wavetable();
    //init_sine_table(); // Initialize to sine wave by default
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
    // command_shell();

    // Initialize voices
    for(int i = 0; i < NVOICES; i++) {
        step[i] = 0;
        offset[i] = 0;
        adsr[i].state = 0;
        adsr[i].count = 0;
        adsr[i].amplitude = 0.0f;
    }

    for(;;) {
        char event = get_key_event();
        char key = event & 0x7F;
        int is_press = event & 0x80;

        if (key == 'A') {
            if (is_press) {
                LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);
                load_waveform_from_sdcard("sine.bin");
                waveform_select(bmp_filenames[0]);
                LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
            }
        } else if (key == 'B') {
            if (is_press) {
                LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);
                load_waveform_from_sdcard("square.bin");
                waveform_select(bmp_filenames[1]);
                LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
            }
        } else if (key == 'C') {
            if (is_press) {
                LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);
                load_waveform_from_sdcard("triangle.bin");
                waveform_select(bmp_filenames[2]);
                LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
            }
        } else if (key == 'D') {
            if (is_press) {
                LCD_DrawString(0, 305, 0xFFFF, 0, loading, 12, 1);
                load_waveform_from_sdcard("sawtooth.bin");
                waveform_select(bmp_filenames[3]);
                LCD_DrawString(0, 305, 0x0000, 0, loading, 12, 1);
            }
        } else {
            // Handle other keys (notes)
            int idx = -1;
            for (int i = 0; i < NVOICES; i++) {
                if (key_note_map[i] == key) {
                    idx = i;
                    break;
                }
            }
            if (idx != -1) {
                if (is_press) {
                    // Start playing note with ADSR envelope
                    if (step[idx] == 0) { // Prevent re-initializing if already playing
                        step[idx] = (int)((note_freqs[idx] * N / RATE) * (1<<16));
                        // Initialize ADSR envelope
                        adsr[idx].state = 1; // Attack
                        adsr[idx].count = 0;
                        adsr[idx].amplitude = 0.0f;
                    }
                } else {
                    // Start release phase
                    if (adsr[idx].state != 0) {
                        adsr[idx].state = 4; // Release
                        adsr[idx].count = 0;
                        // adsr[idx].amplitude remains as is
                    }
                }
            }
        }
    }
}