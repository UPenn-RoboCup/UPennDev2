#ifndef __DTMF_H__
#define __DTMF_H__

#include "pnSequence.h"

// number of audio samples per frame
static const int NUM_SAMPLE = 512;
//#define NUM_SAMPLE 512


/**
 * print out the the magnitude frequency responses for the touch tones
 */
void print_tone_resp(double *qRow, double *qRow2, double *qCol, double *qCol2);

/**
 * find max two elements of an array
 *
 * *x - double array
 *  n - number of elements in array
 * &index - index of the max element is set
 *
 * return the ratio of the max two elements
 */
double sort_ratio(double *x, int n, int &index);


/**
 * ???
 */
void filter_fft(int *x, int *y, int n);


/**
 * compute the standard deviation of the array
 * *x - integer array
 * n - number of elements of x
 *
 * return standard deviation of the array data
 */
double standard_deviation(int *x, int n);


/**
 * ???
 */
int find_first_max(int *x, int n, double threshold, int offset);


/**
 * main update function for incoming audio frames
 *
 * *x - interleaved (lrlrlr) stereo audio signal of size NUM_SAMPLES
 */
void check_tone(short *x);

#endif

