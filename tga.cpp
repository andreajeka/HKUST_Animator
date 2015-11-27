#include "tga.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#define RD16(x) (uint16_t)((uint16_t)x[0] | ((uint16_t)x[1] << 8))

inline int roundpower2(int width)
{
	int b = width;
	int n;
	for (n = 0; b != 0; n++) b >>= 1;
	b = 1 << n;
	if (b == 2 * width) b >>= 1;
	return b;
}

void decodeRLE(void* src, int srcLen, void* dst, int dstLen, int bits)
{
	long count = 0;
	long count2 = 0;
	unsigned char* s = (unsigned char*)src;
	unsigned char* d = (unsigned char*)dst;
	int sswap = 0, dswap = 0;
	unsigned char c;
	unsigned char a;
	unsigned char b1, b2;
	unsigned char t[3];
	unsigned char w[4];

	if (bits == 4) {
		sswap = 0, dswap = 0;
		do {
			if (sswap)
				c = (*(s + count) & 0xF0) | (*(s + count + 1) & 0xF);
			else
				c = *(s + count);
			count++;
			if (c & 0x80) {
				if (sswap ^= 1)
					a = (*(s + count) & 0xF);
				else
					a = (*(s + count++) >> 4);
				c = (c & 0x7F) + 1;
				if (dswap)
				{
					*(d + count2++) |= (a << 4);
					c--;
					dswap = 0;
				}
				for (; c>1; c -= 2)
					*(d + count2++) = (a | a << 4);
				if (c)
				{
					dswap = 1;
					*(d + count2) = a;
				}
			}
			else {
				c++;
				if (sswap == dswap)
				{
					if (sswap)
					{
						*(d + count2++) |= *(s + count++) & 0xF0;
						c--;
					}
					for (; c>1; c -= 2)
						*(d + count2++) = *(s + count++);
					if (c)
					{
						*(d + count2) = *(s + count) & 0xF;
						dswap = sswap = 1;
					}
				}
				else
				{
					for (; c>0; c--)
						if (dswap ^= 1)
							*(d + count2) = *(s + count++) >> 4;
						else
							*(d + count2++) |= *(s + count) << 4;
					sswap = dswap ^ 1;
				}
			}
		} while ((count<srcLen) && (count2<dstLen));


	}
	else if (bits == 8) {
		do {
			c = *(s + count++);
			if (c & 0x80) {
				c = (c & 0x7F);
				a = *(s + count++);
				for (c++; c>0; c--)
					*(d + count2++) = a;
			}
			else {
				for (c++; c>0; c--)
					*(d + count2++) = *(s + count++);
			}
		} while ((count<srcLen) && (count2<dstLen));


	}
	else if (bits == 16) {
		do {
			c = *(s + count++);
			if (c & 0x80) {
				b1 = *(s + count++);
				b2 = *(s + count++);
				c = (c & 0x7F);
				for (c++; c>0; c--) {
					*(d + count2++) = b1;
					*(d + count2++) = b2;
				}
			}
			else {
				for (c++; c>0; c--) {
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
				}
			}
		} while ((count<srcLen) && (count2<dstLen));


	}
	else if (bits == 24) {
		do {
			c = *(s + count++);
			if (c & 0x80) {

				t[0] = *(s + count++);
				t[1] = *(s + count++);
				t[2] = *(s + count++);
				c = (c & 0x7F);
				for (c++; c>0; c--) {
					*(d + count2++) = t[0];
					*(d + count2++) = t[1];
					*(d + count2++) = t[2];
				}
			}
			else {
				for (c++; c>0; c--) {
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
				}
			}
		} while ((count<srcLen) && (count2<dstLen));


	}
	else if (bits == 32) {
		do {
			c = *(s + count++);
			if (c & 0x80) {

				w[0] = *(s + count++);
				w[1] = *(s + count++);
				w[2] = *(s + count++);
				w[3] = *(s + count++);
				c = (c & 0x7F);
				for (c++; c>0; c--) {
					*(d + count2++) = w[0];
					*(d + count2++) = w[1];
					*(d + count2++) = w[2];
					*(d + count2++) = w[3];
				}
			}
			else {
				for (c++; c>0; c--) {
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
					*(d + count2++) = *(s + count++);
				}
			}
		} while ((count<srcLen) && (count2<dstLen));
	}
}

unsigned char* readTGA(const char *fname, int& width, int& height)
{
	FILE* fp;
	unsigned char *pimage = NULL;
	int32_t bytesperline, rlesize;
	uint32_t texw, texh, fpos, color32, col32;
	uint8_t *tdata, *dst, *rle, *src, *xsrc;
	uint8_t done, bpb;
	char TRUEVIS[] = "TRUEVISION-XFILE.";
	long extoffs, devoffs;
	uint32_t *p32;
	uint16_t *p16;
	int x = 0, y = 0, size;
	int32_t ddelta, hdelta, vdelta;
	uint16_t color16;
	static uint8_t r, g, b, a;
	TGAFILEHEADER tfh;
	memset(&tfh, 0, sizeof(TGAFILEHEADER));
	fpos = 0;

	fp = fopen(fname, "rb");
	if (!fp) {
		fprintf(stderr, "File Not Found");
	}

	fseek(fp, 0, SEEK_END);
	int bsize = ftell(fp);
	fseek(fp, 0, SEEK_SET);

	fread(&tfh, sizeof(TGAFILEHEADER), 1, fp);

	fpos += sizeof(TGAFILEHEADER) + tfh.ImageIDSize;

	fseek(fp, fpos, SEEK_SET);

	if (tfh.ImageTypeCode != 1 && tfh.ImageTypeCode != 2 && tfh.ImageTypeCode != 3 &&
		tfh.ImageTypeCode != 9 && tfh.ImageTypeCode != 10 && tfh.ImageTypeCode != 11)
	{
		fprintf(stderr, "Unknown ImageTypeCode.\n");
		return 0;
	}
	if (tfh.Depth != 8 && tfh.Depth != 15 && tfh.Depth != 16 && tfh.Depth != 24 && tfh.Depth != 32)
	{
		fprintf(stderr, "Unknown Bit Depth.\n");
		return 0;
	}

	width = RD16(tfh.Width);// - RD16(tfh.OriginX);
	height = RD16(tfh.Height);// - RD16(tfh.OriginY);
	texw = roundpower2(width);
	texh = roundpower2(height);
	bytesperline = ((width*tfh.Depth) >> 3);
	tdata = (uint8_t*)malloc(height * bytesperline);
	dst = NULL;
	done = 0;
	bpb = 4;
	if (tdata == 0){
		fprintf(stderr, "malloc failed on tdata.\n");
		return 0;
	}

	if ((tfh.ImageTypeCode & 8) == 8 && (tfh.ImageTypeCode & 3) > 0)
	{
		// READ RLE ENCODED
		rlesize = bsize - fpos;
		rle = (uint8_t*)malloc(rlesize);
		if (rle == 0)
		{
			free(tdata);
			return 0;
		}

		fread(rle, rlesize, 1, fp);

		if (memcmp(&rle[rlesize - 26 + 8], TRUEVIS, 18) == 0)
		{
			// TRUEVISION-XFILE and may contain Developer and Extension Areas
			rlesize -= 26;
			extoffs = rle[rlesize + 0] + (rle[rlesize + 1] << 8) + (rle[rlesize + 2] << 16) + (rle[rlesize + 3] << 24);
			devoffs = rle[rlesize + 4] + (rle[rlesize + 5] << 8) + (rle[rlesize + 6] << 16) + (rle[rlesize + 7] << 24);
			if (extoffs != 0 || devoffs != 0)
			{
				// contains developer and/or extension area :/
				// For now we just assume that the developer or extension area is at the start of the whole extension block
				// Actually, we'd need to go through all following area blocks and find the one that comes first (lowest offset)
				if (devoffs<extoffs)
					rlesize = devoffs - fpos;
				else
					rlesize = extoffs - fpos;
			}
		}
		// DECODE RLE:
		decodeRLE(rle, rlesize, tdata, height * bytesperline, tfh.Depth);
		if (rle) { free(rle); rle = NULL; }
	}
	else if ((tfh.ImageTypeCode == 1) || (tfh.ImageTypeCode == 2) || (tfh.ImageTypeCode == 3))
	{
		fread(tdata, bytesperline, height, fp);

	}
	else
	{
		free(tdata);
		fprintf(stderr, "Unknown ImageTypeCode.\n");
		return 0;
	}


	if ((tfh.Depth == 24) || (tfh.Depth == 32)){
		bpb = 4;
		size = texw * texh * bpb;
		dst = (uint8_t*)malloc(size);
		memset(dst, 0, size);
		src = (uint8_t*)tdata;
		ddelta = (texw - width)*bpb;
		hdelta = bytesperline;
		if ((tfh.ImageDescrip & TGA_DESC_HORIZONTAL) == 0){
			src += (height - 1) * bytesperline;
			hdelta = -bytesperline;
		}
		vdelta = (tfh.Depth >> 3);
		if ((tfh.ImageDescrip & TGA_DESC_VERTICAL) != 0){
			src += bytesperline - (tfh.Depth >> 3);
			vdelta = -(tfh.Depth >> 3);
		}

		// convert the crappy image and do flipping if neccessary

		switch (tfh.Depth)
		{
		case 8:
		case 15:
		case 16:
			break;
		case 24:
			p32 = (uint32_t*)dst;
			p16 = (uint16_t*)p32;
			for (y = 0; y < (int)height; y++){
				xsrc = src;
				for (x = 0; x < (int)width; x++)  {

					col32 = (uint32_t)(((uint32_t)xsrc[2] << 16) | ((uint32_t)xsrc[1] << 8) | (uint32_t)xsrc[0] | (0xFF << 24));
					//BGRA
					a = (col32 >> 24) & 0xff;
					b = col32 & 0xff;
					g = (col32 >> 8) & 0xff;
					r = (col32 >> 16) & 0xff;
					color32 = (r | (g << 8) | (b << 16) | (a << 24));
					*(p32 + x) = color32;
					xsrc += vdelta;
				}
				src += hdelta;
				p32 += texw;
				p16 += texw;
			}
			break;
		case 32:
			p32 = (uint32_t*)dst;
			p16 = (uint16_t*)p32;
			for (y = 0; y < (int)height; y++){
				xsrc = src;
				for (x = 0; x < (int)width; x++)  {
					col32 = *(uint32_t*)(xsrc);
					//BGRA
					a = (col32 >> 24) & 0xff;
					b = col32 & 0xff;
					g = (col32 >> 8) & 0xff;
					r = (col32 >> 16) & 0xff;
					color32 = (r | (g << 8) | (b << 16) | (a << 24));
					*(p32 + x) = color32;
					xsrc += vdelta;
				}//end for
				src += hdelta;
				p32 += texw;
				p16 += texw;
			}//end for
		}//end switch
		done = 1;
	}//end if

	if (done){
		pimage = (uint8_t *)dst;
	}

	free(tdata);
	return pimage;
}
