/*
 * Daniel D. Lee <ddlee@seas.upenn.edu> 01/2013
 *
 * fast 12
*/


#ifndef fast12_h
#define fast12_h

typedef unsigned char byte;

int fast12_corner(const byte* im,
		  int xsize, int ysize,
		  int threshold,
		  int xstride, int ystride,
		  int **xp, int **yp);

#endif
