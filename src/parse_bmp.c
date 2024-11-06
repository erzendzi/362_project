#include "parse_bmp.h"
#include "lcd.h"
#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

void read_bmp_header(char *filename, BITMAPFILEHEADER *file_header, BITMAPV5HEADER *info_header) {
    FIL bmp_file;            // File object
    FRESULT fr;             // FatFS return code
    UINT br;                // Bytes read

    fr = f_open(&bmp_file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open BMP file.\n");
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
void draw_bmp_to_lcd(char *filename, BITMAPFILEHEADER *file_header, BITMAPV5HEADER *info_header) {
    FIL bmp_file;
    FRESULT fr;
    UINT br;
    // LCD is 320X240

    lcddev.select(1);
    LCD_direction(2);

    // Open the BMP file
    fr = f_open(&bmp_file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open BMP file.\n");
        LCD_direction(0);
        lcddev.select(0);
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
    //LCD_SetWindow(0, 0, width - 1, height - 1);
    uint8_t offset_x = 0;
    uint8_t offset_y = 40;
    LCD_SetWindow(offset_x, offset_y, width + offset_x - 1, height + offset_y - 1);

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
        //#define OG
        #ifdef OG
        for (int x = 0; x < width; x++) {
            LCD_DrawPoint(x, y, lcd_buffer[x]);
        }
        #endif

        #define ALT
        #ifdef ALT
        LCD_WriteData16_Prepare();
        for (int x = 0; x < width; x++) {
            LCD_WriteData16(lcd_buffer[x]);
        }
        LCD_WriteData16_End();
        
        #endif
    }
    LCD_direction(0);
    lcddev.select(0);
    f_close(&bmp_file);
}

void display_bmp_information(char *filename, BITMAPFILEHEADER *file_header, BITMAPV5HEADER *info_header) {
    FIL bmp_file;
    FRESULT fr;
    UINT br;

    fr = f_open(&bmp_file, filename, FA_READ);
    //if (fr != FR_OK) {
    //    printf("Failed to open BMP file.\n");
    //    return;
    //}

    // Print BMP File Header Information
    printf("BMP File Type: 0x%X\n", file_header->bfType);
    printf("BMP File Size: %u bytes\n", file_header->bfSize);
    printf("BMP Reserved1: %u\n", file_header->bfReserved1);
    printf("BMP Reserved2: %u\n", file_header->bfReserved2);
    printf("BMP Data Offset: %u bytes\n\n", file_header->bfOffBits);

    // Print BMP Info Header Information
    printf("BMP Info Header Size: %u bytes\n", info_header->biSize);
    printf("Image Width: %d pixels\n", info_header->biWidth);
    printf("Image Height: %d pixels\n", info_header->biHeight);
    printf("Number of Planes: %u\n", info_header->biPlanes);
    printf("Bits per Pixel: %u\n", info_header->biBitCount);
    printf("Compression Type: %u\n", info_header->biCompression);
    printf("Image Size: %u bytes\n", info_header->biSizeImage);
    printf("Horizontal Resolution: %d pixels/meter\n", info_header->biXPelsPerMeter);
    printf("Vertical Resolution: %d pixels/meter\n", info_header->biYPelsPerMeter);
    printf("Number of Colors Used: %u\n", info_header->biClrUsed);
    printf("Number of Important Colors: %u\n", info_header->biClrImportant);
}