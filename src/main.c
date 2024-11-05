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

void read_bmp_header(const char *filename, BITMAPFILEHEADER *file_header, BITMAPV5HEADER *info_header) {
    FIL bmp_file;            // File object
    FRESULT fr;             // FatFS return code
    UINT br;                // Bytes read

    // Open the BMP file in binary read mode
    fr = f_open(&bmp_file, filename, FA_READ);
    if (fr != FR_OK) {
        return;
    }

    // Read BITMAPFILEHEADER
    fr = f_read(&bmp_file, file_header, sizeof(BITMAPFILEHEADER), &br);
    if (fr != FR_OK || br != sizeof(BITMAPFILEHEADER)) {
        f_close(&bmp_file);
        return;
    }

    // Validate BMP signature ('BM')
    if (file_header->bfType != 0x4D42) { // 'BM' in little endian
        f_close(&bmp_file);
        return;
    }

    // Read BITMAPV5HEADER
    fr = f_read(&bmp_file, info_header, sizeof(BITMAPV5HEADER), &br);
    if (fr != FR_OK || br != sizeof(BITMAPV5HEADER)) {
        f_close(&bmp_file);
        return;
    }

    // Close the file as we've read the necessary headers
    f_close(&bmp_file);

    return;
}

/**
 * @brief Converts 24-bit RGB color to 16-bit RGB565 format.
 * 
 * @param red   8-bit red component.
 * @param green 8-bit green component.
 * @param blue  8-bit blue component.
 * @return uint16_t RGB565 representation of the color.
 */
uint16_t RGB24_to_RGB565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

/**
 * @brief Draws a BMP image to the LCD.
 * 
 * @param filename     The name of the BMP file to display.
 * @param file_header  Pointer to the BMP file header.
 * @param info_header  Pointer to the BMP info header.
 */
void draw_bmp_to_lcd(const char *filename, BITMAPFILEHEADER *file_header, BITMAPV5HEADER *info_header) {
    FIL bmp_file;
    FRESULT fr;
    UINT br;

    // Open the BMP file
    fr = f_open(&bmp_file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open BMP file.\n");
        return;
    }

    // Move the file pointer to the pixel data offset
    fr = f_lseek(&bmp_file, file_header->bfOffBits);
    if (fr != FR_OK) {
        printf("Failed to seek to pixel data. Error code: %d\n", fr);
        f_close(&bmp_file);
        return;
    }

    // Get image dimensions
    int width = info_header->biWidth;
    int height = info_header->biHeight;
    int bits_per_pixel = info_header->biBitCount;
    int compression = info_header->biCompression;

    if (bits_per_pixel != 24 || compression != 0) {
        printf("Unsupported BMP format. Only 24-bit uncompressed BMPs are supported.\n");
        f_close(&bmp_file);
        return;
    }

    // Calculate the padding for each row (each row is padded to a multiple of 4 bytes)
    int row_stride = (width * 3 + 3) & ~3;

    uint8_t row_buffer[row_stride]; // Buffer to hold one row of BMP data
    uint16_t lcd_buffer[width];     // Buffer to hold one row of RGB565 data

    // Initialize the LCD window to match the BMP size
    LCD_SetWindow(0, 0, width - 1, height - 1);

    // Iterate over each row (BMP files are stored bottom-to-top)
    for (int y = height - 1; y >= 0; y--) {
        fr = f_read(&bmp_file, row_buffer, row_stride, &br);
        if (fr != FR_OK || br != row_stride) {
            printf("Failed to read pixel data. Error code: %d\n", fr);
            f_close(&bmp_file);
            return;
        }

        // Convert each pixel from RGB24 to RGB565
        for (int x = 0; x < width; x++) {
            uint8_t blue  = row_buffer[x * 3];
            uint8_t green = row_buffer[x * 3 + 1];
            uint8_t red   = row_buffer[x * 3 + 2];
            lcd_buffer[x] = RGB24_to_RGB565(red, green, blue);
        }

        // Send the row to the LCD
        //LCD_WriteData16_Prepare();
        for (int x = 0; x < width; x++) {
            LCD_DrawPoint(x, y, lcd_buffer[x]);
        }
        //LCD_WriteData16_End();
    }

    f_close(&bmp_file);
}


int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    
    // Example: Toggle an LED to indicate success (assuming LED is connected to PA5)
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOA clock
    GPIOC->MODER &= ~(0x3 << (6 * 2));  // Set PC6 as output
    GPIOC->MODER |= (0x1 << (6 * 2));  // Set PC6 as output
    GPIOC->ODR ^= (1 << 6);

    LCD_Setup();
    LCD_Clear(0);
    //LCD_DrawFillRectangle(0,0,200,200,0x0f0f);

    mount_sd_card();
    // Define the BMP file name (ensure this file exists on your SD card)
    const char *bmp_filename = "goose.bmp"; // Replace with your BMP file name

    // Structures to hold BMP header information
    BITMAPFILEHEADER file_header;
    BITMAPV5HEADER info_header;

    read_bmp_header(bmp_filename, &file_header, &info_header);
    // LCD is 320X240

    // Print BMP File Header Information
    printf("BMP File Type: 0x%X\n", file_header.bfType);
    printf("BMP File Size: %u bytes\n", file_header.bfSize);
    printf("BMP Reserved1: %u\n", file_header.bfReserved1);
    printf("BMP Reserved2: %u\n", file_header.bfReserved2);
    printf("BMP Data Offset: %u bytes\n\n", file_header.bfOffBits);

// Print BMP Info Header Information
    printf("BMP Info Header Size: %u bytes\n", info_header.biSize);
    printf("Image Width: %d pixels\n", info_header.biWidth);
    printf("Image Height: %d pixels\n", info_header.biHeight);
    printf("Number of Planes: %u\n", info_header.biPlanes);
    printf("Bits per Pixel: %u\n", info_header.biBitCount);
    printf("Compression Type: %u\n", info_header.biCompression);
    printf("Image Size: %u bytes\n", info_header.biSizeImage);
    printf("Horizontal Resolution: %d pixels/meter\n", info_header.biXPelsPerMeter);
    printf("Vertical Resolution: %d pixels/meter\n", info_header.biYPelsPerMeter);
    printf("Number of Colors Used: %u\n", info_header.biClrUsed);
    printf("Number of Important Colors: %u\n", info_header.biClrImportant);


    //LCD_DrawPicture(0, 0, bmp_filename);
    draw_bmp_to_lcd(bmp_filename, &file_header, &info_header);
    //LCD_DrawString(0, 0, 0xFFFF, 0x0000, "I can't get the goose", 18, 0);
    //LCD_DrawString(0, 30, 0xFFFF, 0x0000, "on here but I love you!", 18, 0);
    

    // Indicate success by toggling LED
    //while(1) {
    //    GPIOC->ODR ^= (1 << 6);        // Toggle PA5
    //    for(uint32_t i = 0; i < 100000; i++); // Simple delay
    //}

    //command_shell();
}