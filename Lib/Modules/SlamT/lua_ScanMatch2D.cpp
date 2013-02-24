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
    fprintf(stdout,"ScanMatch2D: set the resolution\n");
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
    fprintf(stdout,"ScanMatch2D: set the boundaries\n");
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
    fprintf(stdout,"ScanMatch2D: set sensor offsets\n");
    return 1;
  }

  if (strcasecmp(command, "match") == 0) 
  {
		// Initialize max finding
			double hmax = 0;
			double xmax = 1; // Lua index
			double ymax = 1; // Lua index
			double thmax = 11; // Lua index
		
	// TODO: Check for NULL pointers
    //luaT_stackdump( L );
	// Get the map, which is a ByteTensor
	// TODO: ensure that the map is explored with the right dimensions
    THByteTensor * map_t = (THByteTensor *) luaT_checkudata(L, 2, "torch.ByteTensor");
    const int sizex = map_t->size[0];
    const int sizey = map_t->size[1];
    const int size  = sizex * sizey;

    /* Grab the xs and ys from the last laser scan*/
    THDoubleTensor * lxs_t = (THDoubleTensor *) luaT_checkudata(L, 3, "torch.DoubleTensor");
    THDoubleTensor * lys_t = (THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
    const int nps = lys_t->size[0]; // The number of laser points to match

    /* Grab the scanning values for theta, x, y */
    THDoubleTensor * pxs_t = (THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
    THDoubleTensor * pys_t = (THDoubleTensor *) luaT_checkudata(L, 6, "torch.DoubleTensor");
    THDoubleTensor * pths_t = (THDoubleTensor *) luaT_checkudata(L, 7, "torch.DoubleTensor");
	int npxs = pxs_t->size[0];
    int npys = pys_t->size[0];
	int npths = pths_t->size[0];
	
	/* Grab the output Tensor */
	THDoubleTensor * likelihoods_t = (THDoubleTensor *) luaT_checkudata(L, 8, "torch.DoubleTensor");
	// TODO: Check that the dimensions are correct
	/*
	fprintf( stdout, "Size of the likelihoods_t: (%d dim): %ld x %ld x %ld\n",
		likelihoods_t->nDimension, 
		likelihoods_t->size[0],likelihoods_t->size[1],likelihoods_t->size[2]
	);
	*/

    // Divide the candidate pose xy by resolution, to save computations later
    // Subtract the min values too
    for (int ii=0; ii<npxs; ii++)
      THTensor_fastSet1d(pxs_t, ii, (THTensor_fastGet1d(pys_t,ii)-ymin)*invRes );
    for (int ii=0; ii<npys; ii++)
      THTensor_fastSet1d(pys_t, ii, (THTensor_fastGet1d(pys_t,ii)-ymin)*invRes );

    for (int ii=0; ii<nps; ii++)
    {
      lxss[ii] = THTensor_fastGet1d(lxs_t,ii) * invRes;
      lyss[ii] = THTensor_fastGet1d(lys_t,ii) * invRes;
    }

    for (int pthi =0; pthi<npths; pthi++)   //iterate over all yaw values
    {
      double costh = cos( THTensor_fastGet1d(pths_t,pthi) );
      double sinth = sin( THTensor_fastGet1d(pths_t,pthi) );
	  //THTensor_fastGet3d( likelihoods_t, pthi, npxs, npys )
      //double * likelihoodsXY = likelihoods + pthi*npxs*npys;

      // Sensor global offset due to robot's yaw and local offsets
      double offsetx = (sensorOffsetX*costh - sensorOffsetY*sinth)*invRes;
      double offsety = (sensorOffsetX*sinth + sensorOffsetY*costh)*invRes;

      // Iterate over all points
      for (int pi=0; pi<nps; pi++)
      {
        // Reset the pointer to the likelyhoods of the poses
        //double * tl = likelihoodsXY;

        // Convert the laser points to the map coordinates
        double xd = THTensor_fastGet1d( lxs_t, pi )*costh - THTensor_fastGet1d( lys_t, pi )*sinth + offsetx;
        double yd = THTensor_fastGet1d( lxs_t, pi )*sinth + THTensor_fastGet1d( lys_t, pi )*costh + offsety;

        // Iterate over all pose ys
        for (int pyi=0; pyi<npys; pyi++) {
          // Use unsigned int - don't have to check < 0
          unsigned int yi  = yd + THTensor_fastGet1d( pys_t, pyi );// + 0.0;

          if (yi >= sizey) {
            // Increment the pointer to likelyhoods by number of x poses
            //tl+=npxs;
            continue;
          }

          //int tmi = yi*sizex;
          //uint8_t * mapp = &(map[tmi]);

          // Iterate over all pose xs
          for (int pxi=0; pxi<npxs; pxi++) {
            //use unsigned int - don't have to check < 0
            //unsigned int xi = xd + *tpxs++;
			unsigned int xi = xd + THTensor_fastGet1d( pxs_t, pxi );
            if (xi >= sizex)
            {
              //tl++;
              continue;
            }

            //*tl++ += mapp[xi];
			double newLikelihood = THTensor_fastGet3d(likelihoods_t,xi,yi,pthi) + (double)THTensor_fastGet2d(map_t,xi,yi);
			// Find the maximum likelihood
			if( newLikelihood > hmax ){
				hmax = newLikelihood;
				xmax = xi+1; // Lua index
				ymax = yi+1; // Lua index
				thmax = pthi+1; // Lua index
			}
			THTensor_fastSet3d(likelihoods_t,xi,yi,pthi,newLikelihood);
//			fprintf(stdout,"Setting values...\n");
			THTensor_fastSet3d(likelihoods_t,xi,yi,pthi,22);
          }
        }
      }
    }
	//luaT_pushudata(L, likelihoods_t, "torch.DoubleTensor");
	//return 1;
		lua_pushnumber(L,hmax);
		lua_pushinteger(L,xmax);
		lua_pushinteger(L,ymax);
		lua_pushinteger(L,thmax);
		return 4;
	//lua_pushlightuserdata(L, likelihoods);
    //lua_pushstring(L, "double");
    //lua_pushinteger(L, npths*npxs*npys);
    //return 3;
  }
  else {
    luaL_error(L ,"unknown command");
		return 0;
	}
}

