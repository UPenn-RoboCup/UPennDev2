/*
* Daniel D. Lee <ddlee@seas.upenn.edu> 01/2013
*
* fast 12
*/

#include "fast_12.h"

#include <vector>

// Bresenham radius 3 neighbors in clockwise order:
const int nNeighbors = 16;
const int neighbors[nNeighbors][2] = {
	{0,3},{1,3},{2,2},{3,1},{3,0},{3,-1},{2,-2},{1,-3},
	{0,-3},{-1,-3},{-2,-2},{-3,-1},{-3,0},{-3,1},{-2,2},{-1,3}
};

// Contiguous 12 neighbor masks
const int signature[nNeighbors] = {
	0xFFF0, 0x7FF8, 0x3FFC, 0x1FFE,
	0x0FFF, 0x87FF, 0xC3FF, 0xE1FF,
	0xF0FF, 0xF87F, 0xFC3F, 0xFE1F,
	0xFF0F, 0xFF87, 0xFFC3, 0xFFE1
};

// Neighbor processing order (reverse binary 0000, 1000, 0100, 1100...)
const int iNeighbor[nNeighbors] =
	{0, 8, 4,12, 2,10, 6,14, 1, 9, 5,13, 3,11, 7,15};
// Minimum count for possible 12-connected neighbors:
const int minNeighbor[nNeighbors] =
	{0, 1, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 9,10,11,12};

int fast12_corner(const byte* im,
int xsize, int ysize,
int threshold,
int xstride, int ystride,
int **xp, int **yp) {

	static std::vector<int> xc, yc;
	xc.clear();
	yc.clear();

	int offset[nNeighbors];
	for (int i = 0; i < nNeighbors; i++) {
		offset[i] = neighbors[i][1]*ystride+neighbors[i][0]*xstride;
	}

	const int boundary = 3;
	for (int iy = boundary; iy < ysize-boundary; iy++) {
		for (int ix = boundary; ix < xsize-boundary; ix++) {
			// array index of center pixel (ix and iy are 0-indexed)
			int icenter = iy*ystride + ix*xstride;
			const byte *c0 = im+icenter;

			int nA = 0, nB = 0;
			unsigned int maskA = 0, maskB = 0;
			for (int j = 0; j < nNeighbors; j++) {
				int i1 = iNeighbor[j];
				const byte *c1 = c0+offset[i1];
				if (*c1 > *c0 + threshold) {
					nA++;
					maskA |= (1 << i1);
				}
				else if (*c1 < *c0 - threshold) {
					nB++;
					maskB |= (1 << i1);
				}
				// Shortcut processing if possible
				if ((nA < minNeighbor[j]) && (nB < minNeighbor[j])) break;
			}

			if (nA >= 12) {
				for (int j = 0; j < nNeighbors; j++) {
					if ( (maskA & signature[j]) == signature[j]) {
						xc.push_back(ix);
						yc.push_back(iy);
						break;
					}
				}
			}
			else if (nB >= 12) {
				for (int j = 0; j < nNeighbors; j++) {
					if ( (maskB & signature[j]) == signature[j]) {
						xc.push_back(ix);
						yc.push_back(iy);
						break;
					}
				}
			}

		}
	}

	int nc = xc.size();
	if (nc > 0) {
		*xp = &xc[0];
		*yp = &yc[0];
	}

	return nc;
}
