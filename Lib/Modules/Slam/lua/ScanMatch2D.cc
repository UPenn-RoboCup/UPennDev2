#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "Timer.hh"
#include <vector>

#define DEFAULT_RESOLUTION 0.05

double xmin,ymin,zmin,xmax,ymax,zmax;
double res = DEFAULT_RESOLUTION;
double invRes = 1.0/res;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double sensorOffsetZ = 0;

double lxss[1081];
double lyss[1081];

vector<double> xss;
vector<double> yss;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const int BUFLEN = 256;
  char command[BUFLEN];

  if (mxGetString(prhs[0], command, BUFLEN) != 0) {
    mexErrMsgTxt("Could not read string. (1st argument)");
  }
  
  
  if (strcasecmp(command, "setResolution") == 0) 
  {
    if (nrhs != 2)
      mexErrMsgTxt("resolution must be provided as the second argument");
    res = *mxGetPr(prhs[1]);
    invRes = 1.0/res;
    printf("ScanMatch2D: set the resolution\n");
    return;
  }

  if (strcasecmp(command, "setBoundaries") == 0) 
  {
    if (nrhs != 5)
      mexErrMsgTxt("a min and max value must be provided for each dimension, 4 total");
    xmin = *mxGetPr(prhs[1]);
    ymin = *mxGetPr(prhs[2]);
    xmax = *mxGetPr(prhs[3]);
    ymax = *mxGetPr(prhs[4]);
    printf("ScanMatch2D: set the boundaries\n");
    return;
  }

  if (strcasecmp(command, "setSensorOffsets") == 0) 
  {
    if (nrhs != 2)
      mexErrMsgTxt("please provide sensor xyz offsets as second argument");

    double * offsets = mxGetPr(prhs[1]);
    sensorOffsetX = offsets[0];
    sensorOffsetY = offsets[1];
    sensorOffsetZ = offsets[2];
    printf("ScanMatch2D: set sensor offsets\n");
    return;
  }

  if (strcasecmp(command, "match") == 0) 
  {
    Upenn::Timer timer0;
    timer0.Tic();

    //make sure that the map is uint8
    if (mxGetClassID(prhs[1]) != mxUINT8_CLASS)
      mexErrMsgTxt("the map must be a uint8 matrix");
  
    uint8_t * map    = (uint8_t*)mxGetData(prhs[1]);
    int nDims        = mxGetNumberOfDimensions(prhs[1]);
    const int * dims = mxGetDimensions(prhs[1]);
    const int sizex  = dims[0];
    const int sizey  = dims[1];
    const int size   = sizex*sizey;
  
    int nlxs = mxGetNumberOfElements(prhs[2]);
    int nlys = mxGetNumberOfElements(prhs[3]);
    int nps  = nlxs;

    if (nlxs != nlys)
      mexErrMsgTxt("arrays with point coordinates must be same length");

    double * lxs   = mxGetPr(prhs[2]);
    double * lys   = mxGetPr(prhs[3]);

    int npxs  = mxGetNumberOfElements(prhs[4]);
    int npys  = mxGetNumberOfElements(prhs[5]);
    int npths = mxGetNumberOfElements(prhs[6]);

    double * pxs   = mxGetPr(prhs[4]);
    double * pys   = mxGetPr(prhs[5]);
    double * pths  = mxGetPr(prhs[6]);

    double * tpxs  = pxs;
    double * tpys  = pys;
    double * tpths = pths;
    double * tlxs  = lxs;
    double * tlys  = lys;

    const int nDimsOut = 3;
    int dimsOut[] = {npxs,npys,npths};    

    plhs[0] = mxCreateNumericArray(nDimsOut,dimsOut,
                              mxDOUBLE_CLASS,mxREAL);
    double * likelihoods = mxGetPr(plhs[0]);

    //resize the container if needed
    xss.resize(npxs);
    yss.resize(npys);

    //get pointers to the temporary arrays
    double * pxss  = &(xss[0]);
    double * pyss  = &(yss[0]);
    
    //divide the candidate pose xy by resolution, to save computations later
    //subtract the min values too
    for (int ii=0; ii<npxs; ii++)
      pxss[ii] = (pxs[ii]-xmin)*invRes; 

    for (int ii=0; ii<npys; ii++)
      pyss[ii] = (pys[ii]-ymin)*invRes;


    for (int ii=0; ii<nps; ii++)
    {
      lxss[ii] = lxs[ii] * invRes;
      lyss[ii] = lys[ii] * invRes;
    }

    tpths = pths;
    for (int pthi =0; pthi<npths; pthi++)   //iterate over all yaw values
    {
      double costh = cos(*tpths);
      double sinth = sin(*tpths);
      tpths++;

      double * likelihoodsXY = likelihoods + pthi*npxs*npys;
      
      //sensor global offset due to robot's yaw and local offsets
      double offsetx = (sensorOffsetX*costh - sensorOffsetY*sinth)*invRes;
      double offsety = (sensorOffsetX*sinth + sensorOffsetY*costh)*invRes;

      tlxs  = lxss;
      tlys  = lyss;

      for (int pi=0; pi<nps; pi++)          //iterate over all points
      {
        double * tl = likelihoodsXY;          //reset the pointer to the likelyhoods of the poses

        //convert the laser points to the map coordinates
        double xd = (*tlxs)*costh   - (*tlys)*sinth   + offsetx;
        double yd = (*tlxs++)*sinth + (*tlys++)*costh + offsety;
        
        tpys = pyss;
        for (int pyi=0; pyi<npys; pyi++)    //iterate over all pose ys
        {
          //use unsigned int - don't have to check < 0
          unsigned int yi  = yd + *tpys++;// + 0.0;

          if (yi >= sizey)
          {
            tl+=npxs;                       //increment the pointer to likelyhoods by number of x poses
            continue;
          }

          int tmi = yi*sizex;
          uint8_t * mapp = &(map[tmi]);
    
          tpxs = pxss;
          for (int pxi=0; pxi<npxs; pxi++)  //iterate over all pose xs
          {
            //use unsigned int - don't have to check < 0
            unsigned int xi = xd + *tpxs++;
            if (xi >= sizex)
            {
              tl++;
              continue;
            }

            *tl++ += mapp[xi];
          }
        }
      }
    }
    //timer0.Toc(true);
  }

  else 
    mexErrMsgTxt("unknown command");
}

