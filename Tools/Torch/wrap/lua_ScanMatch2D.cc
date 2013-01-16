#ifdef __cplusplus
extern "C" {
#endif

#include "torch/lua.h"
#include "torch/lualib.h"
#include "torch/lauxlib.h"
#include "torch/luaT.h"

#ifdef __cplusplus
}
#endif

#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <vector>
#include "Timer.hh"

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

int lua_ScanMatch2D(lua_State *L) {
  //  const int BUFLEN = 256;
  const char *command = luaL_checkstring(L, 1);

  if (command == NULL)
    luaL_error(L, "Could not read string. (1st argument)");

  if (strcasecmp(command, "setResolution") == 0) 
  {
    if lua_isnoneornil(L, 2)
      luaL_error(L, "resolution must be provided as the second argument");
    res = luaL_checknumber(L, 2);
    invRes = 1.0/res;
    printf("ScanMatch2D: set the resolution\n");
    return 1;
  }

  if (strcasecmp(command, "setBoundaries") == 0) 
  {
    if lua_isnoneornil(L, 5)
      luaL_error(L, "a min and max value must be provided for each dimension, 4 total");
    xmin = luaL_checknumber(L, 2);
    ymin = luaL_checknumber(L, 3);
    xmax = luaL_checknumber(L, 4);
    ymax = luaL_checknumber(L, 5);
    printf("ScanMatch2D: set the boundaries\n");
    return 1;
  }

  if (strcasecmp(command, "setSensorOffsets") == 0) 
  {
    if (lua_isnoneornil(L, 2))
      luaL_error(L, "please provide sensor xyz offsets as second argument");

    double * offsets = (double *)lua_touserdata(L, 2);
    if ((offsets == NULL) || !lua_islightuserdata(L, 2)) {
      return luaL_error(L, "Input offsets not light user data");
    }
    sensorOffsetX = offsets[0];
    sensorOffsetY = offsets[1];
    sensorOffsetZ = offsets[2];
    printf("ScanMatch2D: set sensor offsets\n");
    return 1;
  }

  if (strcasecmp(command, "match") == 0) 
  {
    Upenn::Timer timer0;
    timer0.Tic();
		    luaT_stackdump( L );

    // Make sure that the map is uint8
    // TODO: check that it is a ByteStorage
    const char* name = luaT_typename( L, 2 );
    //printf("Name: [%s]\n", name);
    uint8_t * map = (uint8_t *) luaT_toudata(L, 2, name);
    if ( map==NULL || !luaT_isudata(L, 2, name) )
      return luaL_error(L, "Input map not uint8 light user data");
    
    int nDims = luaL_checkint(L, 3); 
    const int sizex = luaL_checkint(L, 4);
    const int sizey = luaL_checkint(L, 5);
    const int size   = sizex*sizey;
    printf("\nSize of the Map: %d x %d. %d elements\n",sizex,sizey,size);
    
    const char* nameX = luaT_typename( L, 6 );
    double * lxs = (double *) luaT_toudata(L, 6, nameX);
    if ( lxs==NULL || !luaT_isudata(L, 6, name) )
      return luaL_error(L, "Input lxs not double light user data");
    
    const char* nameY = luaT_typename( L, 7 );
    double * lys = (double *) luaT_toudata(L, 7, nameY);
    if ( lys==NULL || !luaT_isudata(L, 7, name) )
      return luaL_error(L, "Input lys not double light user data");
		
//		luaT_getfieldchecktable(L, 6, "size");
//		int tmp = luaT_getfieldcheckint(L, 6, "size");
//		printf("Size: %d\n",tmp);

    int nlxs = sizeof(lxs) / sizeof(double);
    int nlys = sizeof(lys) / sizeof(double);
    int nps  = nlxs;
    printf("\nNumber of points: [%d]\n",nps);

    if (nlxs != nlys)
      luaL_error(L, "arrays with point coordinates must be same length");

    double * pxs = (double *) luaT_toudata(L, 8, luaT_typename( L, 8 ));
    if ( pxs==NULL || !luaT_isudata(L, 8, luaT_typename( L, 8 )) )
      return luaL_error(L, "Input pxs not double light user data");
		
    double * pys = (double *) luaT_toudata(L, 9, luaT_typename( L, 9 ));
    if ( pys==NULL || !luaT_isudata(L, 9, luaT_typename( L, 9 )) )
      return luaL_error(L, "Input pys not double light user data");

    double * pths = (double *) luaT_toudata(L, 10, luaT_typename( L, 10 ));
    if ( pths==NULL || !luaT_isudata(L, 10, luaT_typename( L, 10 )) )
      return luaL_error(L, "Input pths not double light user data");

    int npxs = sizeof(pxs) / sizeof(double);
    int npys = sizeof(pys) / sizeof(double);
    int npths = sizeof(pths) / sizeof(double);

    double * tpxs  = pxs;
    double * tpys  = pys;
    double * tpths = pths;
    double * tlxs  = lxs;
    double * tlys  = lys;

    const int nDimsOut = 3;
    int dimsOut[] = {npxs,npys,npths};    

    //    plhs[0] = mxCreateNumericArray(nDimsOut,dimsOut,
    //                              mxDOUBLE_CLASS,mxREAL);
    //    double * likelihoods = mxGetPr(plhs[0]);
    double * likelihoods = new double[npxs * npys * npths];

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
    lua_pushlightuserdata(L, likelihoods);
    lua_pushstring(L, "double");
    lua_pushinteger(L, npths*npxs*npys);
    return 3;
  }
  else 
    luaL_error(L ,"unknown command");
}

