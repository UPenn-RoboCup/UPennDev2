#include <lua.hpp>
#include <torch/luaT.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <torch/TH/TH.h>
#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <list>
#include <stdbool.h>
#include <float.h>
#ifdef DEBUG
#include <iostream>
#endif

#define CAPACITY 200 // TODO
#define CUBOID_DEPTH 0.02 // meters
#define DEFAULT_RESOLUTION 0.02
#define DEFAULT_INV_RESOLUTION 50

/* Candidate indices helpers for speed */
unsigned int xis[1081];
unsigned int yis[1081];
/* Laser points in map coordinates */
double lxs_map[1081];
double lys_map[1081];

// For updating the map
const int range_max = 255;
const int range_min = 0;

double xmin, ymin, xmax, ymax;
double invxmax, invymax;
int nrow, ncol;

int depth = CUBOID_DEPTH;
double res = DEFAULT_RESOLUTION, invRes = DEFAULT_INV_RESOLUTION;

typedef struct CUBOID {
  int pnum; // # of points registered
  int cls;  // class label
  float h, d;  // height and depth of the cuboid
  std::list<int> msnlist;
  float p[3], P[3][3], s[3], S[3][3];
} CUBOID;


/* Set the resolution of the map */
int lua_set_resolution(lua_State *L) {
	res = luaL_checknumber(L, 1);
	invRes = 1.0 / res;
	return 0;
}

/* Set the boundaries for scan matching and map updating */
int lua_set_boundaries(lua_State *L) {
	xmin = luaL_checknumber(L, 1);
	ymin = luaL_checknumber(L, 2);
	xmax = luaL_checknumber(L, 3);
	ymax = luaL_checknumber(L, 4);
	invymax = ymax * DEFAULT_INV_RESOLUTION;
	invxmax = xmax * DEFAULT_INV_RESOLUTION;
  nrow = floor((ymax-ymin)*invRes + 0.5);
  ncol = floor((xmax-xmin)*invRes + 0.5);
	return 0;
}

/* Set up empty map */
// TODO: list or vector inside each cell?
// we need to insert in a sorted list
// and we also need constatnly random access...
static std::vector<std::vector<CUBOID> > map;
int lua_set_up_map(lua_State *L) {
  map.resize(nrow*ncol);
  return 0;
}

// Add a point and update the cuboid accordingly
void add_point(std::vector<CUBOID> cell, int z) {
  for (int i=0; i<cell.size(); i++) {
    if (cell[i].h-cell[i].d>z) {
      // Point is not within or close to any cuboid
      CUBOID c;
      c.h = z;
      c.d = depth;
      c.pnum = 1;
      cell.insert(cell.begin()+i, i, c);
      break;
      
    } else if (cell[i].h>=z && cell[i].h-cell[i].d<=z) {
      // Point is IN current cuboid
      cell[i].d = cell[i].d + cell[i].h - z;
      break;
    } else if (cell[i].h<z && cell[i].h+depth>=z) {
      // Point is above current cuboid but close
      cell[i].h = z;
      cell[i].d = cell[i].d + z - cell[i].h;
      break;
    } else if (i==cell.size()-1) {
      // hit the last one already and need a new cuboid
      CUBOID c;
      c.h = z;
      c.d = depth;
      c.pnum = 1;
      cell.push_back(c);
      break;
    } 
  }
}

void update_part_map(int x) {
  
}

int lua_grow_map (lua_State *L) {
  printf("TCCM MAP UPDATING...\n");
  
	THFloatTensor *scan_t = (THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	THArgCheck(scan_t->nDimension == 2, 1, "tensor must have two dimensions");
  
  /*
  printf("dim0: %lo \t dim1: %lo \n", 
    scan_t->size[0], scan_t->size[1]);
  printf("stide0: %lo \t stride1: %lo \n", 
    scan_t->stride[0], scan_t->stride[1]);
  */
  
	// Check contiguous
	THArgCheck(scan_t->stride[1] == 1, 1, "Improper memory layout (j)");
	const long scan_istride = scan_t->stride[0];
	THArgCheck(scan_istride == scan_t->size[1], 1, "Improper memory layout (i)"); 
  
  // Project the points into cuboids
  float x, y, z;
  int xi, yi, map_idx;
  float* scan_ptr = scan_t->storage->data ; //TODO?: + scan_t->storageOffset;
  std::vector<std::vector<CUBOID> > part_map(nrow*ncol);
  for (int i=0; i<scan_t->size[1]; i++) {
    x = *(scan_ptr);
    y = *(scan_ptr + scan_istride);
    z = *(scan_ptr + 2*scan_istride);
    scan_ptr += 1;
    // Boundary check
		if(x<xmin||x>xmax||y<ymin||y>ymax) continue;
    
		// Floor so that it starts from 0
		xi = floor((x - xmin) * invRes);
		yi = floor((y - ymin) * invRes);
		// Get the index on the map
		map_idx = xi * scan_istride + yi;
    
    // update cuboid in this cell
    if (part_map[map_idx].empty()) {
      // Create a new cuboid
      CUBOID c;
      c.pnum = 1;
      c.h = z;
      c.d = depth;
      // TODO: insert as needed
      part_map[map_idx].push_back(c);
    } else {
      // check if fall into an existing cuboid
      add_point(part_map[map_idx], z);
    }
    
  }
  return 0;
}


/* Update the map with the laser scan points */
int lua_update_map(lua_State *L) {

	/* Get the map, which is a ByteTensor */
	const THByteTensor * map_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
	// Check contiguous
	THArgCheck(map_t->stride[1] == 1, 1, "Improper memory layout (j)");
	const long map_istride = map_t->stride[0];
	THArgCheck(map_istride == map_t->size[1], 1, "Improper memory layout (i)");

	/* Grab the updated points */
	const THDoubleTensor * pts_t =
		(THDoubleTensor *) luaT_checkudata(L, 2, "torch.DoubleTensor");
	THArgCheck(pts_t->size[1]==2, 2, "Proper laser points");

	/* Grab the increment value */
	const int inc = luaL_checknumber(L, 3);

	/* Get the updating table: sets which items were updated */
	const THDoubleTensor * update_t =
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	THArgCheck(update_t->stride[1] == 1, 4, "Improper memory layout (j)");
	THArgCheck(update_t->stride[0] == update_t->size[1], 4, "Improper memory layout (i)");
	THArgCheck(map_istride == update_t->stride[0], 4, "Improper memory layout (i)");
	
	/* Grab the timestamp */
	const double t = luaL_checknumber(L, 5);

	double * update_ptr = (double *)(update_t->storage->data + update_t->storageOffset);
	uint8_t * map_ptr = (uint8_t *)(map_t->storage->data + map_t->storageOffset);
	double* pts_ptr = (double *)(pts_t->storage->data + pts_t->storageOffset);

	const long nps  = pts_t->size[0];
	const int pts_stride = pts_t->stride[0];

	/*
	// TODO: Check the map size...?
	const long sizex = map_t->size[0];
	const long sizey = map_t->size[1];
	*/

	double x, y;
	int i, xi, yi, mapLikelihood;
	long map_idx;
	double * update_ptr_tmp;
	uint8_t* map_ptr_tmp;
	for( i=0; i<nps; i++ ) {
		// Grab the cartesian coordinates of the point
		x = *(pts_ptr);
		y = *(pts_ptr+1);
		pts_ptr += pts_stride;

		// If out of range...
		if( x>xmax || y>ymax || x<xmin || y<ymin ) continue;

		/* TODO: ceil or floor these? map_t bounds check? */
		xi = ( x - xmin ) * invRes;
		yi = ( y - ymin ) * invRes;

		// Get the index on the map (same as update map)
		map_idx = xi * map_istride + yi;
		update_ptr_tmp = update_ptr + map_idx;

		/* Check if this cell has been updated before */
		if( *update_ptr_tmp >=t ) continue;

		// Find the current likelihood
		map_ptr_tmp = map_ptr + map_idx;
		mapLikelihood = *map_ptr_tmp + inc;

		/* --------------------------- */
		/* bit hack for range limit */
		/* http://graphics.stanford.edu/~seander/bithacks.html#IntegerMinOrMax */
		/* newVal = min(mapLikelihood, 255) */
		mapLikelihood = range_max ^ ((mapLikelihood ^ range_max) & -(mapLikelihood < range_max));
		mapLikelihood = mapLikelihood ^ ((mapLikelihood ^ range_min) & -(mapLikelihood < range_min));
		/* --------------------------- */

		// Update the maps
		*map_ptr_tmp = mapLikelihood;
		*update_ptr_tmp = t;
	}
	return 0;
}

/* Decay the map around a region from the robot  */
int lua_decay_map(lua_State *L) {

	/* Get the torch that contains the map data */
	/* NOTE: This is a sub-map around the robot... stride is important now! */
	const THByteTensor * map_t =
		(THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
	uint8_t * map_ptr = (uint8_t *)(map_t->storage->data + map_t->storageOffset);

	/* Grab the threshold on likelihood */
	const double thres = luaL_checknumber(L, 2);

	/* Grab by how much to decay the map */
	const double decay_factor = luaL_checknumber(L, 3);

	/* Get the map properties */
	const long ystride = map_t->stride[1];
	THArgCheck(ystride == 1, 1, "Improper memory layout (j)");
	const long xstride = map_t->stride[0];
	const long nps_x = map_t->size[0];
	const long nps_y = map_t->size[1];

	uint8_t* cur_map = map_ptr;
	int i=0, j=0;
	uint8_t val = 0;

	// Loop through the submap
	for( i=0; i<nps_x; i++ ) {
		for ( j=0; j<nps_y; j++){
			val = *(cur_map+j);
			if ( val>=thres ){
				/* If super certain: remain at high level */
				*(cur_map+j) = thres;
			} else {
				/* Decay */
				*(cur_map+j) = val * decay_factor;
			}
		} /* for j */
		cur_map += xstride;
	} /* for i */

	return 0;
}


/************************************************************************/



static const struct luaL_Reg tccm_lib [] = {
	{"grow_map", lua_grow_map},
	//
	{"update_map", lua_update_map},
	{"decay_map", lua_decay_map},
	//
	{"set_resolution", lua_set_resolution},
	{"set_boundaries", lua_set_boundaries},
  {"set_up_map", lua_set_up_map},
	//
	{NULL, NULL}
};


#ifdef __cplusplus
extern "C"
#endif

int luaopen_tccm (lua_State *L) {

#if LUA_VERSION_NUM == 502
	luaL_newlib(L, tccm_lib);
#else
	luaL_register(L, "tccm", tccm_lib);
#endif
	return 1;
}
