/* 
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

#include <lua.hpp>
#include "jpeg_utils.h"
#include <string.h>
#define MT_NAME "jpeg_mt"

std::vector<unsigned char> destBuf;
static uint8_t global_shift = 8;
#define MAX_CH 4
#define MAX_WIDTH 640
//static JSAMPLE row[MAX_CH*MAX_WIDTH];
static int quality = 85;

static structJPEG * lua_checkjpeg(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid jpeg");
  return (structJPEG *)ud;
}

inline JSAMPLE* assign_data(const uint8_t *p, JSAMPLE *row, int width, 
                int height, int ch, int scale) {
	/*
  int irow = 0;
  for (int i = 0; i < width; i++) {
    for (int c = 0; c < ch; c++)
      row[irow++] = *(p + i * ch + c);
  }
	*/
	
	memcpy ( row, p, ch*width );
	return row;
	
	//return (JSAMPLE*)p;
}

inline JSAMPLE* assign_data_16(const uint8_t *p, JSAMPLE *row, int width, 
                int height, int ch, int scale) {
	const uint16_t *p16 = (uint16_t*)p;
  for (int i = 0; i < width; i++)
      row[i] = (uint8_t)( (p16[i] >> global_shift) & 0x00FF );
	return row;
}

inline JSAMPLE* assign_data_yuyv(const uint8_t *p, JSAMPLE *row, int width, 
                int height, int ch, int scale) {
  int row_ind = 0;
  uint8_t Y0 = 0, Y1 = 0, U = 0, V = 0;
  while (row_ind < width * ch) {
    Y0 = *(p++);
    U  = *(p++);
    Y1 = *(p++);
    V  = *(p++);
    row[row_ind++] = Y0;  
    row[row_ind++] = U;  
    row[row_ind++] = V;  
    if (scale == 1) {
      row[row_ind++] = Y1;  
      row[row_ind++] = U;  
      row[row_ind++] = V;
    } else if (scale == 2) {
    } else if (scale == 4) {
      p+=4;
    } else {
      fprintf(stdout, "Scale rate not support!!!\n");
    }
  }
	return row;
}

int CompressData(const uint8_t* datap, int width, int height, int data_ch, int ch, int scale,
                J_COLOR_SPACE color_space, 
                JSAMPLE*(*assign_data)(const uint8_t *p, JSAMPLE *row, 
                                        int width, int height, int ch, int scale)) {
  int out_width = width / scale;
  int out_height = height / scale;

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jerr.error_exit = error_exit_compress;

  jpeg_create_compress(&cinfo);
  if (cinfo.dest == NULL) {
    cinfo.dest = (struct jpeg_destination_mgr *)
      (*cinfo.mem->alloc_small) ((j_common_ptr) &cinfo, JPOOL_PERMANENT,
          sizeof(struct jpeg_destination_mgr));
  }
  cinfo.dest->init_destination = init_destination;
  cinfo.dest->empty_output_buffer = empty_output_buffer;
  cinfo.dest->term_destination = term_destination;

  cinfo.image_width = out_width;
  cinfo.image_height = out_height;
  cinfo.input_components = ch;
  cinfo.in_color_space = color_space;

  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  //cinfo.write_JFIF_header = true;
  cinfo.write_JFIF_header = false;
  //cinfo.dct_method = JDCT_IFAST;
  cinfo.dct_method = JDCT_FASTEST; // TurboJPEG

  jpeg_start_compress(&cinfo, TRUE);

	int stride = data_ch * width * (scale / 2 + 1); 

  JSAMPLE* row = new JSAMPLE[ch*out_width];
  JSAMPROW row_pointer[1];
  *row_pointer = row;
	uint8_t* p = (uint8_t*)datap;
  while (cinfo.next_scanline < cinfo.image_height) {
		*row_pointer = assign_data(p, row, out_width, out_height, cinfo.input_components, scale);
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
		
		p+=stride;
  }
	delete row;
  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  unsigned int destBufSize = destBuf.size();
  return destBufSize;		
}

static int lua_jpeg_compress(lua_State *L) {
  luaL_error(L, "compress method deprecated\n use compress_[rgb|yuv|yuyv|gray] instead\n");
  return 1;
}

static int lua_set_quality(lua_State *L) {
	int q = luaL_checkint(L, 1);
	if(q<1||q>98)
  	return luaL_error(L, "Quality must be in range of 1 to 98");
	quality = q;
  return 1;
}

static int lua_jpeg_compress_rgb(lua_State *L) {
  uint8_t * data = NULL;
  size_t sz = 0;
  if (lua_isstring(L, 1)) {
    data = (uint8_t *) lua_tolstring(L, 1, &sz); 
  } else if (lua_islightuserdata(L, 1)) {
    data = (uint8_t *) lua_touserdata(L, 1);
  } else {
    lua_pushnil(L);
    return 1;
  }

  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);

  int scale = 1;

  int lenPacked = CompressData(data, width, height, 3, 3, scale, JCS_RGB, assign_data);
  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}

static int lua_jpeg_compress_yuv(lua_State *L) {
  uint8_t * data = NULL;
  size_t sz = 0;
  if (lua_isstring(L, 1)) {
    data = (uint8_t *) lua_tolstring(L, 1, &sz); 
  } else if (lua_islightuserdata(L, 1)) {
    data = (uint8_t *) lua_touserdata(L, 1);
  } else {
    lua_pushnil(L);
    return 1;
  }

  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  int scale = 1;

  int lenPacked = CompressData(data, width, height, 3, 3, scale, JCS_YCbCr, assign_data);
  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}

static int lua_jpeg_compress_yuyv(lua_State *L) {
  uint8_t * data = NULL;
  size_t sz = 0;
  // 1st input : yuyv lightuserdata or lstring
  if (lua_isstring(L, 1)) {
    data = (uint8_t *) lua_tolstring(L, 1, &sz); 
  } else if (lua_islightuserdata(L, 1)) {
    data = (uint8_t *) lua_touserdata(L, 1);
  } else {
    lua_pushnil(L);
    return 1;
  }
  // 2nd input : width
  int width = luaL_checkint(L, 2);
  // 3rd input : height
  int height = luaL_checkint(L, 3);
  // 4th input : subsampling rate, default 1
  int scale = luaL_optinteger(L, 4, 1);

  // 32bits for 2 pixel = 2byte for 1 pixel
  int lenPacked = CompressData(data, width, height, 2, 3, scale, JCS_YCbCr, assign_data_yuyv);

  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}

static int lua_jpeg_compress_gray(lua_State *L) {
  uint8_t * data = NULL;
  size_t sz = 0;
  if (lua_isstring(L, 1)) {
    data = (uint8_t *) lua_tolstring(L, 1, &sz); 
  } else if (lua_islightuserdata(L, 1)) {
    data = (uint8_t *) lua_touserdata(L, 1);
  } else {
    lua_pushnil(L);
    return 1;
  }
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  int scale = 1;

  int lenPacked = CompressData(data, width, height, 1, 1, scale, JCS_GRAYSCALE, assign_data);
  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}

static int lua_jpeg_compress_16(lua_State *L) {
  uint8_t * data = NULL;
  size_t sz = 0;
  if (lua_isstring(L, 1)) {
    data = (uint8_t *) lua_tolstring(L, 1, &sz); 
  } else if (lua_islightuserdata(L, 1)) {
    data = (uint8_t *) lua_touserdata(L, 1);
  } else {
		return luaL_error(L, "Bad JPEG Compress 16 input");
  }
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  int scale = 1;
	
	/* How much to shift the 16bits to fit into an 8bit grayscale pixel */
	int shift_amt = luaL_optint (L, 4, 8);
	if( shift_amt > 8 || shift_amt < 0 )
		return luaL_error(L, "Bad JPEG Compress 16 input");
	global_shift = shift_amt;

  int lenPacked = CompressData(data, width, height, 1, 1, scale, JCS_GRAYSCALE, assign_data_16);
  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}

static int lua_jpeg_uncompress(lua_State *L) {
  char* file_str = (char*)luaL_checkstring(L, 1);
  int size = lua_tointeger(L, 2);

  structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
  
  struct jpeg_decompress_struct cinfo;
  struct mem_error_mgr jerr;

  jpeg_create_decompress(&cinfo);
  mem_jpeg_source_mgr mjsm( (char*) file_str, size );
  cinfo.src = &mjsm;

  cinfo.err = jpeg_std_error(&jerr);
  jerr.error_exit = mem_error_exit;

  if (!setjmp(jerr.setjmp_buffer)) {
    jpeg_read_header( &cinfo, TRUE );

    ud->height = cinfo.image_height;
    ud->width = cinfo.image_width;
    ud->stride = cinfo.num_components;

	  ud->raw_image = (unsigned char*)malloc( cinfo.image_width*cinfo.image_height*cinfo.num_components );

    jpeg_start_decompress( &cinfo );

    unsigned char* buffer = ud->raw_image;
    unsigned int dstStep = cinfo.output_width*cinfo.num_components;

    while (cinfo.output_scanline < cinfo.output_height) 
    {
      jpeg_read_scanlines(&cinfo, (JSAMPARRAY) &buffer, 1);
      buffer += dstStep;
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
  }

  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);

  return 1;
}

static int lua_jpeg_getValue(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index
  if ((index < 0) || (index >= p->height * p->stride)) {
    lua_pushnil(L);
    return 1;
  }
  lua_pushinteger(L, p->raw_image[index]);

  return 1;
}

static int lua_jpeg_delete(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  /* cleanup heap allocation */
  free(p->raw_image);
  return 1;
}

static int lua_jpeg_setValue(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index
  if ((index < 0) || (index >= p->height * p->stride)) {
    lua_pushnil(L);
    return 1;
  }

  int val = lua_tointeger(L, 3);
  
  p->raw_image[index] = val;
  return 1;
}

static int lua_jpeg_index(lua_State *L) {
  lua_checkjpeg(L, 1);
  if ((lua_type(L, 2) == LUA_TNUMBER) && lua_tointeger(L, 2)) {
    // Numeric index:
    return lua_jpeg_getValue(L);
  }

  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_jpeg_width(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushinteger(L, p->width);
  return 1;
}

static int lua_jpeg_height(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushinteger(L, p->height);
  return 1;
}

static int lua_jpeg_stride(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushinteger(L, p->stride);
  return 1;
}

static int lua_jpeg_len(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushinteger(L, p->width * p->height * p->stride);
  return 1;
}

static int lua_jpeg_pointer(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushlightuserdata(L, ((unsigned char*)p->raw_image));
  return 1;
}

static int lua_jpeg_tostring(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushlstring(L, (const char *)p->raw_image, 
                          p->width * p->height * p->stride);
  return 1;
}

static const struct luaL_reg jpeg_Functions [] = {
	{"set_quality", lua_set_quality},
  {"compress", lua_jpeg_compress},
  {"compress_rgb", lua_jpeg_compress_rgb},
  {"compress_gray", lua_jpeg_compress_gray},
	{"compress_16", lua_jpeg_compress_16},
  {"compress_yuv", lua_jpeg_compress_yuv},
  {"compress_yuyv", lua_jpeg_compress_yuyv},
  {"uncompress", lua_jpeg_uncompress},
  {NULL, NULL}
};

static const struct luaL_reg jpeg_Methods [] = {
  {"pointer", lua_jpeg_pointer},
//  {"read", lua_jpeg_read},
//  {"write", lua_jpeg_write},
  {"width", lua_jpeg_width},
  {"height", lua_jpeg_height},
  {"stride", lua_jpeg_stride},
  {"__gc", lua_jpeg_delete},
  {"__newindex", lua_jpeg_setValue},
  {"__tostring", lua_jpeg_tostring},
  {"__index", lua_jpeg_index},
  {"__len", lua_jpeg_len},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_jpeg (lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, jpeg_Methods, 0);
  luaL_newlib(L, jpeg_Functions);
#else
  luaL_register(L, NULL, jpeg_Methods);
  luaL_register(L, "jpeg", jpeg_Functions);
#endif

  return 1;
}
