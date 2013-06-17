/**
 * Simple torch utilities to emulate masking and filtering
 * (c) Stephen McGill, 2013
 * University of Pennsylvania
 * 2010
 */
#include "lua.h"
#include "lauxlib.h"
#include "torch/luaT.h"
#include "torch/TH/TH.h"

static const size_t FLOAT_SZ = sizeof(float);

/**************
 * memcpy
 * memcpy a pointer or string to a pointer location
 * tutil.memcpy( POINTER, STRING/USERDATA, NBYTES )
 */
int lua_memcpy(lua_State *L) {

	/* Destination Pointer */
	void* dest = lua_touserdata(L, 1);
	if( dest == NULL ) {
		return luaL_error(L, "Input src is NULL");
	}

	/* Src pointer and length */
	void* src = NULL;
	size_t nBytes = 0;
	if( lua_islightuserdata(L,2) ) {
		src = lua_touserdata(L, 2);
		if( src == NULL ) {
			return luaL_error(L, "Input src is NULL");
		}
		/* Number of bytes to copy is mandatory for userdata */
		nBytes = luaL_checkinteger( L, 3 );
	} else {
		src = (void*)luaL_checklstring( L, 2, &nBytes );
	}

	/* Copy the data */
	memcpy( dest, src, nBytes );

	/* Do not push anything */
	return 0;
}

/**************
 * memcpy_memcpy_ranges_fov
 * memcpy lidar ranges within a certain field of view
 * tutil.memcpy_ranges_fov( TENSOR, STRING, START_IDX, STOP_IDX )
 */
int lua_memcpy_ranges_fov(lua_State *L) {

	/* Destination lidar ranges Tensor */
	THFloatTensor * data_t = 
		(THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	/* Destination Offset Included */
	size_t offset = data_t->storageOffset;
	/* NOTE: This is dangerous, as it assumes that you know what you are doing */
	float* dest = (float*)(data_t->storage->data) + offset;

	/* Dimension Check */
	size_t nDim = data_t->nDimension;
	if( nDim != 1 ) {
		return luaL_error(L, "Destination must be 1 dimensional!");
	}

	/* Input string */
	size_t nBytes = 0;
	const float * src = (const float *)luaL_checklstring( L, 2, &nBytes );

	/* Minimum angle index (NOTE: Convert from Lua index to C index) */
	size_t mindex = luaL_checkinteger(L, 3) - 1;
	size_t maxdex = luaL_checkinteger(L, 4) - 1;
//	size_t nPoints = maxdex - mindex;
// SJ: fixed here
	size_t nPoints = maxdex - mindex + 1;

	/* Check that there is enough space */
	size_t available_sz = data_t->size[0];
	if( nPoints > available_sz ){
		return luaL_error(L, "Tensor is not big enough");
	}

	size_t nPointBytes = nPoints * FLOAT_SZ;
	if( nPointBytes > nBytes ){
		return luaL_error(L, "Source string is not big enough");
	}

	/* Perform the memory copy */
	memcpy( dest, src+mindex, nPointBytes );

	/* Do not push anything */
	return 0;

}


/**************
 * band_mask(points,min,max)
 * This function removes, in memory, points outside of the min/max band
 * Performs a resize
 * */
int lua_band_mask(lua_State *L) {
	THDoubleTensor * data_t = 
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	THArgCheck(data_t->nDimension == 1, 1, "data should be 1 dimensional");
	const long nElements = data_t->size[0];
	const double min = luaL_checknumber(L,2);
	const double max = luaL_checknumber(L,3);

	int i = 0, curIndex = 0;
	double point = 0;
	for(;i<nElements;i++){
		point = THTensor_fastGet1d(data_t,i);
		if(point>=min && point<=max && i!=curIndex){
			THTensor_fastSet1d(data_t,curIndex++,point);
		}
	}
	// Resize
	THDoubleTensor_resize1d(data_t,curIndex);
	return 0;
}





//SJ: just made a new function here

/**************
 * memcpy_byte_ranges_fov
 * memcpy lidar ranges within a certain field of view
 * AS BYTE data (0-255) given the RANGE
 * tutil.memcpy_ranges_fov( TENSOR, STRING, START_IDX, STOP_IDX ,RANGE )
 */
int lua_byte_cpy_ranges_fov(lua_State *L) {

  /* Destination lidar ranges Tensor */
  THByteTensor * data_t = 
    (THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
  /* Destination Offset Included */
  size_t offset = data_t->storageOffset;
  /* NOTE: This is dangerous, as it assumes that you know what you are doing */
  char* dest = (char*)(data_t->storage->data) + offset;

  /* Dimension Check */
  size_t nDim = data_t->nDimension;
  if( nDim != 1 ) {
    return luaL_error(L, "Destination must be 1 dimensional!");
  }

  /* Input string */
  size_t nBytes = 0;
  const float* src = (const float *)luaL_checklstring( L, 2, &nBytes );

  /* Minimum angle index (NOTE: Convert from Lua index to C index) */
  size_t mindex = luaL_checkinteger(L, 3) - 1;
  size_t maxdex = luaL_checkinteger(L, 4) - 1;

  //Range
  double range = luaL_checknumber(L, 5);
  size_t nPoints = maxdex - mindex + 1;

  /* Check that there is enough space */
  size_t available_sz = data_t->size[0];
  if( nPoints > available_sz ){
    return luaL_error(L, "Tensor is not big enough");
  }

  int i,count = 0;

  for(i=mindex;i<=maxdex;i++){
    float dist = src[i];
    if (dist>range) dist = range;
    dest[count] =  (unsigned char) (dist/range*256.0);


//testing
//    dest[count] = (unsigned char) ((i-mindex)*256/(maxdex-mindex));


    count = count + 1;
  }

  /* Do not push anything */
  return 0;
}


//SJ: Added this for depth image handling
/**************
 * clip_mask(points,min,max)
 * This function clips the points within min/max band
*/
int lua_clip_mask(lua_State *L) {
  THFloatTensor * data_t = 
    (THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
  THArgCheck(data_t->nDimension == 1, 1, "data should be 1 dimensional");
  const long nElements = data_t->size[0];
  const double min = luaL_checknumber(L,2);
  const double max = luaL_checknumber(L,3);

  int i = 0, curIndex = 0;
  double point = 0;
  for(;i<nElements;i++){
    point = THTensor_fastGet1d(data_t,i);
    if (point>max) point = max;
    if (point<min) point = min;
    THTensor_fastSet1d(data_t,i,point);
  }
  // Resize
  return 0;
}












/**************
 * band_mask_key(key,min,max,value1,value2)
 * This function removes, in memory, points outside of the min/max band
 * Removes the points from both value1 and value2, but not key
 * Does not perform a resize
 * */
int lua_band_mask_key(lua_State *L) {
	static int i,curIndex;
	static float key;
	THFloatTensor * k_t = 
		(THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	THArgCheck(k_t->nDimension == 1, 1, "Keys should be 1 dimensional");

	// The key must lie within min and max
	const double min = luaL_checknumber(L,2);
	const double max = luaL_checknumber(L,3);

	/* 
		 Grab the Value tensors
		 Check that Key and Value tensors have the same number of elements
		 */
	THDoubleTensor * v1_t = 
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	THArgCheck(v1_t->nDimension == 1, 4, "Data1 should be 1 dimensional");
	const long nKeys = k_t->size[0];
	const long nData1 = v1_t->size[0];
	if(nKeys!=nData1)
		return luaL_error(L, "Keys and Data1 dimension mismatch");

	THDoubleTensor * v2_t = 
		(THDoubleTensor *) luaT_checkudata(L, 5, "torch.DoubleTensor");
	THArgCheck(v2_t->nDimension == 1, 5, "Data2 should be 1 dimensional");
	const long nData2 = v2_t->size[0];
	if(nKeys!=nData2)
		return luaL_error(L, "Keys and Data2 dimension mismatch");

	for(i = 0, curIndex = 0; i<nKeys; i++){
		key = THTensor_fastGet1d(k_t,i);
		if(key>=min && key<=max) {
			//printf("%lf %lf %lf\n",key,min,max);
			fflush(stdout);
			if(i!=curIndex){
				THTensor_fastSet1d( v1_t, curIndex, THTensor_fastGet1d(v1_t,i) );
				THTensor_fastSet1d( v2_t, curIndex, THTensor_fastGet1d(v2_t,i) );
			}
			curIndex++;
		}
	}

	// Return the new size of the array, without resizing
	lua_pushinteger(L,curIndex);
	return 1;
}

/**************
 * band_mask_key_points(key,min,max,points)
 * This function removes, in memory, points outside of the min/max band
 * Removes the points from both value1 and value2, but not key
 * Does perform a resize
 * */
int lua_band_mask_key_points(lua_State *L) {
	static int i, curIndex;
	static float key;

	THDoubleTensor * k_t = 
		(THDoubleTensor *) luaT_checkudata(L, 1, "torch.DoubleTensor");
	THArgCheck(k_t->nDimension==1, 1, "Keys should be 1 dimensional");

	/* The key must lie within min and max */
	const double min = luaL_checknumber(L,2);
	const double max = luaL_checknumber(L,3);

	/* Grab the points tensor */
	THDoubleTensor * p_t = 
		(THDoubleTensor *) luaT_checkudata(L, 4, "torch.DoubleTensor");
	THArgCheck(p_t->nDimension == 2, 4, "Points should be 2 dimensional");
	THArgCheck(p_t->size[1] == 4, 4, "Points should be of size nx4");

	/* Check that both have the same number of elements */
	const long nKeys = k_t->size[0];
	const long nPoints = p_t->size[0];
	if(nKeys!=nPoints)
		return luaL_error(L, "Keys and Points dimension mismatch");

	for(i=0,curIndex=0;i<nKeys;i++){
		key = THTensor_fastGet1d(k_t,i);
		if(key>=min && key<=max){
			if(i!=curIndex){
				THTensor_fastSet2d( p_t, curIndex, 0, THTensor_fastGet2d(p_t,i,0) );
				THTensor_fastSet2d( p_t, curIndex, 1, THTensor_fastGet2d(p_t,i,1) );
				THTensor_fastSet2d( p_t, curIndex, 2, THTensor_fastGet2d(p_t,i,2) );
				/* 4th not needed yet */
				/*
					 THTensor_fastSet2d( p_t, curIndex, 4, THTensor_fastGet2d(p_t,i,4) );
					 */
			}
			curIndex++;
		}
	}

	/* Resize the resulting points tensor */
	THDoubleTensor_resize2d(p_t, curIndex, 4);
	return 0;
}

static const struct luaL_Reg tutil_lib [] = {
	{"memcpy_ranges_fov", lua_memcpy_ranges_fov},
	{"band_mask_key_points", lua_band_mask_key_points},
	{"band_mask", lua_band_mask},
	{"band_mask_key", lua_band_mask_key},
	{"memcpy", lua_memcpy},
	{"clip_mask", lua_clip_mask},
	{"byte_cpy_ranges_fov", lua_byte_cpy_ranges_fov},
	{NULL, NULL}
};

int luaopen_tutil (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, tutil_lib);
#else
	luaL_register(L, "tutil", tutil_lib);
#endif
	return 1;
}
