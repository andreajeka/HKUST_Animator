#ifndef TGA_H
#define TGA_H

#include <stdint.h>

// ImageType Codes
#define TGA_TYPE_MAPPED		1
#define TGA_TYPE_COLOR		2
#define TGA_TYPE_GRAY		3
#define TGA_TYPE_MAPPED_RLE	9
#define TGA_TYPE_COLOR_RLE	10
#define TGA_TYPE_GRAY_RLE	11

/* Image descriptor:
3-0: attribute (alpha) bpp
4:   left-to-right ordering
5:   top-to-bottom ordering
7-6: zero
*/
// Image Description Bitmasks
#define TGA_DESC_ABITS		0x0f			// Alpha Bits
#define TGA_DESC_HORIZONTAL	0x10			// Left-Right Ordering: 0 = left to right, 1 = right to left
#define TGA_DESC_VERTICAL	0x20			// Top-Bottom Ordering: 0 = bottom to top, 1 = top to bottom

#pragma pack(push)
#pragma pack(1)


typedef struct {
	uint8_t	ImageIDSize;
	uint8_t	ColorMapType;
	uint8_t	ImageTypeCode;                          // Image Type (normal/paletted/grayscale/rle)
	uint8_t	ColorMapOrigin[2];
	uint8_t	ColorMapLength[2];                      // Palette Size
	uint8_t	ColorMapESize;							// Size in bits of one Palette entry
	uint8_t	OriginX[2];
	uint8_t	OriginY[2];
	uint8_t	Width[2];                               // Width of Image
	uint8_t	Height[2];                              // Height of Image
	uint8_t	Depth;                                  // Bits per Pixel of Image
	uint8_t	ImageDescrip;
} TGAFILEHEADER;


#pragma pack(pop)

extern unsigned char *readTGA(const char *fname, int& width, int& height);

#endif
