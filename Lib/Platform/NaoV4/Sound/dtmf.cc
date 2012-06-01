#include "dtmf.h"
#include "fft.h"

// number of tone frequencies (rows and columns)
const short NUM_FREQUENCY = 4;
// frequency corresponding to each row (low fq) of tone symbols
const short K_ROW[]  = {22, 25, 27, 30};
// frequency corresponding to each column (high fq) of tone symbols
const short K_COL[]  = {39, 43, 47, 52};
// characters representing each tone
const char TONE_SYMBOL[4][4] = {{'1','2','3','A'},
                                {'4','5','6','B'},
                                {'7','8','9','C'},
                                {'*','0','#','D'}};

// TODO: what does the NFFT_MULTIPLIER represent?
const int NFFT_MULTIPLIER = 2;
// number of elements in the result from the normalized fft 
const int NFFT = NFFT_MULTIPLIER * PFRAME;

// TODO: what do these thresholds mean
const double THRESHOLD_RATIO1 = 2;
const double THRESHOLD_RATIO2 = 2;
const int THRESHOLD_COUNT = 3;
const int NUM_CHIRP_COUNT = 4;
const int NCORRELATION = NUM_CHIRP_COUNT * PFRAME;

// TODO: frame number should not be here, temporary for matlab testing
long frameNumber = 0;


void print_tone_resp(double *qRow, double *qRow2, double *qCol, double *qCol2) {
  printf("qRow:\t");
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    printf("% 4.3f  ", qRow[i]);
  }
  printf("\n");
  printf("qRow2:\t");
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    printf("% 4.3f  ", qRow2[i]);
  }
  printf("\n");

  printf("qCol:\t");
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    printf("% 4.3f  ", qCol[i]);
  }
  printf("\n");
  printf("qCol2:\t");
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    printf("% 4.3f  ", qCol2[i]);
  }
  printf("\n");

}


double sort_ratio(double *x, int n, int &index) {
  double xMax = 1, xNext = 1;
  index = 0;

  for (int i = 0; i < n; i++) {
    if (x[i] > xMax) {
      index = i;
      xNext = xMax;
      xMax = x[i];
    } else if (x[i] > xNext) {
      xNext = x[i];
    }
  }

  return (xMax / xNext);
}


void filter_fft(int *x, int *y, int n) {
  static int xFilter[NFFT], yFilter[NFFT];
  static bool init = false;

  if (!init) {
    for (int i = 0; i < PFRAME; i++) {
      // time reverse to filter for correlation
      xFilter[i] = pnSequence[PFRAME-1-i]; 
      yFilter[i] = 0;
    }
    for (int i = PFRAME; i < NFFT; i++) {
      xFilter[i] = 0; 
      yFilter[i] = 0;
    }
    fft(xFilter, yFilter, NFFT);
  }

  for (int i = 0; i < n; i++) {
    // complex multiplication of filter FFT
    int tempr = x[i], tempi = y[i];
    x[i] = tempr*xFilter[i] - tempi*yFilter[i];
    y[i] = tempr*yFilter[i] + tempi*xFilter[i];

    // normalization
    /*
    x[i] /= sqrt(tempr*tempr+tempi*tempi+1.0);
    y[i] /= sqrt(tempr*tempr+tempi*tempi+1.0);
    */

    // not necessary--even playing sound from
    //   own speaker results in correlation < 32768
    // if not normalized, rescale to prevent overflows
    x[i] /= 32768/(2*NFFT);
    y[i] /= 32768/(2*NFFT);
  }

  // take inverse fft to calculate convolution
  fft(x, y, n, -1);
}


double standard_deviation(int *x, int n) {
  double xStd = 0.0;
  for (int i = 0; i < n; i++) {
    xStd += (x[i] * x[i]);
  }
  xStd /= n;
  xStd = sqrt(xStd);

  return xStd;
}


int find_first_max(int *x, int n, double threshold, int offset) {
  int i = 0;

  while (i < (n-offset)) {
    if ((x[i] > threshold) && (x[i+offset] < -threshold)) {
      while ((x[i+1] >= x[i]) && (i < n)) {
        i += 1;
      }

      return i;
    }

    i += 1;
  }

  return -1;
}


void check_tone(short *x)  {
  static int toneCount = 0;
  static char prevSymbol = '\0';
  static long startFrame = 0;

  static int xL[NFFT], yL[NFFT], xR[NFFT], yR[NFFT];
  static int leftCorr[NCORRELATION], rightCorr[NCORRELATION];

  double qRow[NUM_FREQUENCY], qRow2[NUM_FREQUENCY];
  double qCol[NUM_FREQUENCY], qCol2[NUM_FREQUENCY];
  int kLRow, kLCol, kRRow, kRCol;
  double rowRatio, colRatio;

  // extract left/right channels
  for (int i = 0; i < PFRAME; i++) {
    xL[i] = x[2*i]; 
    yL[i] = 0;
    xR[i] = x[2*i+1]; 
    yR[i] = 0;
  }
  for (int i = PFRAME; i < NFFT; i++) {
    xL[i] = 0; 
    yL[i] = 0;
    xR[i] = 0; 
    yR[i] = 0;
  }

  // compute fft of left channel
  fft(xL, yL, NFFT);

  // compute the magnitude of the frequency response for each tone
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    // iRow is the array index corresponding to the row (low) frequency
    int iRow = NFFT_MULTIPLIER * K_ROW[i];
    qRow[i] = xL[iRow]*xL[iRow] + yL[iRow]*yL[iRow];
    qRow2[i] = xL[2*iRow]*xL[2*iRow] + yL[2*iRow]*yL[2*iRow];

    // iCow is the array index corresponding to the column (high) frequency
    int iCol = NFFT_MULTIPLIER * K_COL[i];
    qCol[i] = xL[iCol]*xL[iCol] + yL[iCol]*yL[iCol];
    qCol2[i] = xL[2*iCol]*xL[2*iCol] + yL[2*iCol]*yL[2*iCol];
  }

  //print_tone_resp(qRow, qRow2, qCol, qCol2);

  // find ratio of max 2 elements
  //  kLRow and kLCol are indicies of max elements
  rowRatio = sort_ratio(qRow, 4, kLRow);
  colRatio = sort_ratio(qCol, 4, kLCol);

  // are we are still waiting for the tone signal?
  if (toneCount < THRESHOLD_COUNT) {
    // check the expected tone frequency response is within the threshold
    //  max frequency response is less than 2 times greater than the second highest?
    //  TODO: what is qRow2
    if ((rowRatio < THRESHOLD_RATIO1)
        || (qRow[kLRow] < THRESHOLD_RATIO2*qRow2[kLRow])
        || (colRatio < THRESHOLD_RATIO1)
        || (qCol[kLCol] < THRESHOLD_RATIO2*qCol2[kLCol])) {

      toneCount = 0;
      return;
    }
  }

  // compute fft of right channel
  fft(xR, yR, NFFT);

  // compute the magnitude of the frequency response for each tone
  for (int i = 0; i < NUM_FREQUENCY; i++) {
    // iRow is the array index corresponding to the row (low) frequency
    int iRow = NFFT_MULTIPLIER * K_ROW[i];
    qRow[i] = xR[iRow]*xR[iRow] + yR[iRow]*yR[iRow];
    qRow2[i] = xR[2*iRow]*xR[2*iRow] + yR[2*iRow]*yR[2*iRow];

    // iCow is the array index corresponding to the column (high) frequency
    int iCol = NFFT_MULTIPLIER * K_COL[i];
    qCol[i] = xR[iCol]*xR[iCol] + yR[iCol]*yR[iCol];
    qCol2[i] = xR[2*iCol]*xR[2*iCol] + yR[2*iCol]*yR[2*iCol];
  }

  //print_tone_resp(qRow, qRow2, qCol, qCol2);

  // find ratio of max 2 elements
  //  kRRow and kRCol are indicies of max elements
  rowRatio = sort_ratio(qRow, 4, kRRow);
  colRatio = sort_ratio(qCol, 4, kRCol);

  if (toneCount < THRESHOLD_COUNT) {
    if ((rowRatio < THRESHOLD_RATIO1)
        || (kRRow != kLRow)
        || (qRow[kRRow] < THRESHOLD_RATIO2*qRow2[kRRow])
        || (colRatio < THRESHOLD_RATIO1)
        || (kRCol != kLCol)
        || (qCol[kRCol] < THRESHOLD_RATIO2*qCol2[kRCol])) {

      toneCount = 0;
      return;
    }
  }

  // get tone symbol
  char symbol = TONE_SYMBOL[kLRow][kLCol];
  printf("symbol: %c\n", symbol);

  //printf("toneCount: %d\n", toneCount);

  // is this the first tone?
  //  or has the tone changed before expected
  if ((toneCount == 0) 
      || ((toneCount < THRESHOLD_COUNT) && (symbol != prevSymbol))) {
    prevSymbol = symbol;
    toneCount = 1;
    return;
  }

  toneCount++;

  if (toneCount == THRESHOLD_COUNT) {
    // we have heard the tone for the expected amount of time
    //  TODO: what is the significance of the start frame
    //  TODO: is this 4 supposed to be NUM_CHIRP_COUNT
    startFrame = frameNumber + 4;
  } else if (toneCount > THRESHOLD_COUNT) {
    // compute cross correlation of the left and right channels
    filter_fft(xL, yL, NFFT);
    filter_fft(xR, yR, NFFT);

    int nCorr = toneCount - (THRESHOLD_COUNT+1);
    if (nCorr > 0) {
      for (int i = 0; i < PFRAME-1; i++) {
        leftCorr[(nCorr-1)*PFRAME+i+1] += xL[i];
        rightCorr[(nCorr-1)*PFRAME+i+1] += xR[i];
      }
    }
    for (int i = 0; i < PFRAME; i++) {
      leftCorr[nCorr*PFRAME+i] = xL[PFRAME-1+i];
      rightCorr[nCorr*PFRAME+i] = xR[PFRAME-1+i];
    }

    /*
    int nPcm = NUM_SAMPLE*(toneCount-4);
    for (int i = 0; i < NUM_SAMPLE; i++) {
      pcmArray[nPcm+i][0] = x[2*i];
      pcmArray[nPcm+i][1] = x[2*i+1];
    }
    */
  }

  // have we reached the end of the expected audio signal?
  if (toneCount == THRESHOLD_COUNT+NUM_CHIRP_COUNT) {
    printf("DTMF: frameNumber = %ld, tone = '%c'\n", startFrame, prevSymbol);

    /*
    // store correlation in shared memory
    for (int i = 0; i < NCORRELATION; i++) {
      pcmArray(i, 0) = leftCorr[i];
      pcmArray(i, 1) = rightCorr[i];
    }
    */

    // compute standard deviation of the left/right correlation
    double xLStd = standard_deviation(leftCorr, NCORRELATION);
    double xRStd = standard_deviation(rightCorr, NCORRELATION);

    // find first max zero crossing
    const double stdThreshold = 3;
    int xLIndex = find_first_max(leftCorr, NCORRELATION, stdThreshold*xLStd, PFRAME);
    int xRIndex = find_first_max(rightCorr, NCORRELATION, stdThreshold*xRStd, PFRAME);

    // finished cross correlating the stereo signal
    //  get data out to localization modules
    const int strBufLength = 1024;
    char strBuf[strBufLength];
    snprintf(strBuf, strBufLength, "Tone(%c, %ld, %d, %d)\n", prevSymbol, startFrame, xLIndex, xRIndex);
    printf("%s",strBuf);

    // not sure what this is doing
    //  looks like some event driven thing where a method subscribes to the 
    //  correslation data
    /*
    subject[sbjInterpreter]->SetData(strBuf, strlen(strBuf));
    subject[sbjInterpreter]->NotifyObservers();

    subject[sbjDecoder]->SetData(pcmArray);
    subject[sbjDecoder]->NotifyObservers();
    */

    // reset tone count
    toneCount = 0;
  }
}

