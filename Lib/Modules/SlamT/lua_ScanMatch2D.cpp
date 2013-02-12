#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "luaT.h"
#include "TH/TH.h"
#ifdef __cplusplus
}
#endif

#include "lua_ScanMatch2D.h"

using namespace std;

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
    luaT_stackdump( L );

    // Make sure that the map is uint8
    // TODO: check that it is a ByteStorage
    const char* name = luaT_typename( L, 2 );
    //printf("\nMap name: [%s]\n", name);

    THByteTensor * map_t = (THByteTensor *) luaT_checkudata(L, 2, name);
    if ( map_t==NULL || !luaT_isudata(L, 2, name) )
      return luaL_error(L, "Input map not Torch userdata");

    THByteStorage * map_s = map_t->storage;
    /*
       printf("\n%dD Map Size: (%ld)x(%ld)=[%ld]\n",
       map_t->nDimension,
       map_t->size[0],
       map_t->size[1],
       map_s->size
       );
       */
    uint8_t * map = map_s->data;
    // TODO: ensure that the map is explore with the right dimensions
    // MATLAB may malloc rows/columns in a different way than torch
    const int sizex = map_t->size[0];
    const int sizey = map_t->size[1];
    const int size  = sizex * sizey;

    /* Grab the xs */
    THDoubleTensor * lxs_t = (THDoubleTensor *) luaT_checkudata(L, 3, "torch.DoubleTensor");
	printf("xs: %lf %lf\n", THTensor_fastGet1d(lxs_t,0), THTensor_fastGet1d(lxs_t,1) );

    /* Grab the ys */
    THDoubleTensor * lys_t = (THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	// From THTensorMacros.h
	printf("ys: %lf %lf\n", THTensor_fastGet1d(lys_t,0), THTensor_fastGet1d(lys_t,1) );

    /* Account for the number of laser points to match */
    const int nps = lys_t->size[0];

    /* Grab the scanning values for theta, x, y */
    const char* name_px = luaT_typename( L, 5 );
    //printf("Name px: %s\n", name_px);
    THDoubleTensor * pxs_t = (THDoubleTensor *) luaT_toudata(L, 5, name_px);
    if ( pxs_t==NULL || !luaT_isudata(L, 5, name_px) )
      return luaL_error(L, "Input pxs not Torch userdata");
    THDoubleStorage * pxs_s = pxs_t->storage;
    double * pxs = pxs_s->data;
    int npxs = pxs_t->size[0];
    //printf("Number of px elements: [%d],[%lf]\n",npxs,pxs[0]);

    THDoubleTensor * pys_t = (THDoubleTensor *) luaT_toudata(L, 6, luaT_typename( L, 6 ));
    if ( pys_t==NULL || !luaT_isudata(L, 6, luaT_typename( L, 6 )) )
      return luaL_error(L, "Input pys not double light user data");
    THDoubleStorage * pys_s = pys_t->storage;
    double * pys = pys_s->data;
    int npys = pys_t->size[0];
    //printf("Number of py elements: [%d],[%lf]\n",npys,pys[0]);

    THDoubleTensor * pths_t = (THDoubleTensor *) luaT_toudata(L, 7, luaT_typename( L, 7 ));
    if ( pths_t==NULL || !luaT_isudata(L, 7, luaT_typename( L, 7 )) )
      return luaL_error(L, "Input pths not double light user data");
    THDoubleStorage * pths_s = pths_t->storage;
    double * pths = pths_s->data;
    int npths = pths_t->size[0];
    //printf("Number of pth elements: [%d],[%lf]\n",npths,pths[0]);

		// TODO: just use access functions
    double * tpxs  = pxs;
    double * tpys  = pys;
    double * tpths = pths;
    double * tlxs  = NULL;
    double * tlys  = NULL;

    const int nDimsOut = 3;
    int dimsOut[] = {npxs,npys,npths};    

    //    plhs[0] = mxCreateNumericArray(nDimsOut,dimsOut,
    //                              mxDOUBLE_CLASS,mxREAL);
    //    double * likelihoods = mxGetPr(plhs[0]);
    double * likelihoods = new double[npxs * npys * npths];

    // Resize the container if needed
    xss.resize(npxs);
    yss.resize(npys);

    // Get pointers to the temporary arrays
    double * pxss  = &(xss[0]);
    double * pyss  = &(yss[0]);

    // Divide the candidate pose xy by resolution, to save computations later
    // Subtract the min values too
    for (int ii=0; ii<npxs; ii++)
      pxss[ii] = (pxs[ii]-xmin)*invRes; 

    for (int ii=0; ii<npys; ii++)
      pyss[ii] = (pys[ii]-ymin)*invRes;

    for (int ii=0; ii<nps; ii++)
    {
      lxss[ii] = THTensor_fastGet1d(lxs_t,ii) * invRes;
      lyss[ii] = THTensor_fastGet1d(lys_t,ii) * invRes;
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

      // Iterate over all points
      for (int pi=0; pi<nps; pi++)
      {
        // Reset the pointer to the likelyhoods of the poses
        double * tl = likelihoodsXY;

        // Convert the laser points to the map coordinates
        double xd = (*tlxs)*costh   - (*tlys)*sinth   + offsetx;
        double yd = (*tlxs++)*sinth + (*tlys++)*costh + offsety;

        tpys = pyss;
        // Iterate over all pose ys
        for (int pyi=0; pyi<npys; pyi++)
        {
          // Use unsigned int - don't have to check < 0
          unsigned int yi  = yd + *tpys++;// + 0.0;

          if (yi >= sizey)
          {
            // Increment the pointer to likelyhoods by number of x poses
            tl+=npxs;
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
		// TODO: convert to a torch object? or just convert using utility function?
    lua_pushlightuserdata(L, likelihoods);
    lua_pushstring(L, "double");
    lua_pushinteger(L, npths*npxs*npys);
    return 3;
  }
  else {
    luaL_error(L ,"unknown command");
		return 0;
	}
}

