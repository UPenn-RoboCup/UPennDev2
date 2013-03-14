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

double xmin,ymin,zmin;
double xmax,ymax,zmax;
double res = DEFAULT_RESOLUTION;
double invRes = 1.0/res;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double sensorOffsetZ = 0;

int lua_ScanMatch2D(lua_State *L) {
  // luaT_stackdump( L );
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
    return 0;
  }

  if (strcasecmp(command, "setSensorOffsets") == 0) 
  {
    if (lua_isnoneornil(L, 2))
      luaL_error(L, "please provide sensor xyz offsets as second argument");

    sensorOffsetX = luaL_checknumber(L, 2);
    sensorOffsetY = luaL_checknumber(L, 3);
    sensorOffsetZ = luaL_checknumber(L, 4);

    fprintf(stdout,"ScanMatch2D: set sensor offsets\n");
    return 0;
  }

  if (strcasecmp(command, "match") == 0) 
  {
    /* Initialize max correlation value */
    double hmax = 0;
    /* Initialize the max correlation Lua indices */
    unsigned long ixmax = 1; // Lua index
    unsigned long iymax = 1; // Lua index
    unsigned long ithmax = 1; // Lua index
    
    /* Get the map, which is a ByteTensor */
    const THByteTensor * map_t = (THByteTensor *) luaT_checkudata(L, 2, "torch.ByteTensor");
    const int sizex = map_t->size[0];
    const int sizey = map_t->size[1];
    const int size  = sizex * sizey;

    /* Grab the xs and ys from the last laser scan*/
    const THFloatTensor * lY_t = (THFloatTensor *) luaT_checkudata(L, 3, "torch.FloatTensor");
    /* The number of laser points to match */
    const long nps = lY_t->size[1]; 
    /* Grab the scanning values for theta, x, y */
    const THFloatTensor * pxs_t = (THFloatTensor *) luaT_checkudata(L, 4, "torch.FloatTensor");
    const THFloatTensor * pys_t = (THFloatTensor *) luaT_checkudata(L, 5, "torch.FloatTensor");
    const THFloatTensor * pths_t = (THFloatTensor *) luaT_checkudata(L, 6, "torch.FloatTensor");
    long npxs = pxs_t->size[0];
    long npys = pys_t->size[0];
    long npths = pths_t->size[0];

    /* Grab the output Tensor */
    THDoubleTensor * likelihoods_t = (THDoubleTensor *) luaT_checkudata(L, 7, "torch.DoubleTensor");
    if (likelihoods_t->size[0] != npxs || likelihoods_t->size[1]!=npys || likelihoods_t->size[2]!=npths)
      return luaL_error(L, "Likelihood output wrong");

    for (int pthi=0; pthi<npths; pthi++)   //iterate over all yaw values
    {
      double costh = cos( THTensor_fastGet1d(pths_t,pthi) );
      double sinth = sin( THTensor_fastGet1d(pths_t,pthi) );
      // Sensor global offset due to robot's yaw and local offsets
      double offsetx = (sensorOffsetX*costh - sensorOffsetY*sinth)*invRes;
      double offsety = (sensorOffsetX*sinth + sensorOffsetY*costh)*invRes;

      // Iterate over all laser points
      for (int pi=0; pi<nps; pi++) {

        // Convert the laser points to the map coordinates
double lx = THTensor_fastGet2d( lY_t, 1, pi) * invRes;
double ly = THTensor_fastGet2d( lY_t, 2, pi) * invRes;
        double xd = lx*costh - ly*sinth + offsetx;
        double yd = lx*sinth + ly*costh + offsety;
//fprintf(stdout,"xd, yd: %lf, %lf\n",xd,yd);

        // Iterate over all pose ys
        for (int pyi=0; pyi<npys; pyi++) {
          // Use unsigned int - don't have to check < 0
          unsigned int yi  = yd + (THTensor_fastGet1d(pys_t,pyi)-ymin)*invRes;// + 0.0;
          if (yi >= sizey)
            continue;

          // Iterate over all pose xs
          for (int pxi=0; pxi<npxs; pxi++) {
            // Use unsigned int - don't have to check < 0
            unsigned int xi = xd + (THTensor_fastGet1d(pxs_t,pxi)-xmin)*invRes;
            if (xi >= sizex)
              continue;
//fprintf(stdout,"xi, yi: %u %u\n",xi,yi);
double currentLikelihood = THTensor_fastGet3d(likelihoods_t,pxi,pyi,pthi);
uint8_t mapLikelihood = THTensor_fastGet2d(map_t,xi,yi);
double newLikelihood = currentLikelihood+mapLikelihood;
THTensor_fastSet3d(likelihoods_t,pxi,pyi,pthi,newLikelihood);



            // Find the maximum likelihood
            if( newLikelihood > hmax ){
//fprintf(stdout,"New max? %.2lf+%d vs %.2lf.\n",currentLikelihood,mapLikelihood,hmax);
//fflush( stdout );
              hmax = newLikelihood;
              ixmax = pxi+1; // Lua index
              iymax = pyi+1; // Lua index
              ithmax = pthi+1; // Lua index
            }
            
          } // For pose xs
        } // For pose ys
      } // for laser scan points
    } // For yaw values
    lua_pushnumber(L,hmax);
    lua_pushinteger(L,ixmax);
    lua_pushinteger(L,iymax);
    lua_pushinteger(L,ithmax);
    return 4;
  }
  else {
    luaL_error(L ,"unknown command");
    return 0;
  }
}

