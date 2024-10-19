#include <stdio.h>
#include <stdint.h>

// Ensure structures are packed without padding
#pragma pack(push, 1)

// BMP File Header (14 bytes)
typedef struct {
    uint16_t bfType;      // File type ("BM")
    uint32_t bfSize;      // Size of the file in bytes
    uint16_t bfReserved1; // Reserved; must be zero
    uint16_t bfReserved2; // Reserved; must be zero
    uint32_t bfOffBits;   // Offset to start of pixel data
} BITMAPFILEHEADER;

// BMP Info Header (124 bytes for BITMAPV5HEADER)
typedef struct {
    uint32_t biSize;              // Size of this header (124 bytes)
    int32_t  biWidth;             // Image width
    int32_t  biHeight;            // Image height
    uint16_t biPlanes;            // Number of color planes
    uint16_t biBitCount;          // Bits per pixel
    uint32_t biCompression;       // Compression type
    uint32_t biSizeImage;         // Image size (bytes)
    int32_t  biXPelsPerMeter;     // Pixels per meter X
    int32_t  biYPelsPerMeter;     // Pixels per meter Y
    uint32_t biClrUsed;           // Number of colors used
    uint32_t biClrImportant;      // Number of important colors
    // V5 specific fields
    uint32_t bV5RedMask;
    uint32_t bV5GreenMask;
    uint32_t bV5BlueMask;
    uint32_t bV5AlphaMask;
    uint32_t bV5CSType;
    uint8_t  bV5Endpoints[36];
    uint32_t bV5GammaRed;
    uint32_t bV5GammaGreen;
    uint32_t bV5GammaBlue;
    uint32_t bV5Intent;
    uint32_t bV5ProfileData;
    uint32_t bV5ProfileSize;
    uint32_t bV5Reserved;
} BITMAPV5HEADER;

#pragma pack(pop)