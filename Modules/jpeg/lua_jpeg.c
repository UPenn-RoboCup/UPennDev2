/* 
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

// TODO: Add torch support, then we can compress a sub window of the camera image
// If we can add torch support to the uvc library - should be a simple thing with pointers

#include <lauxlib.h>
#include <lua.h>
#include <jpeglib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// UDP Friendly size (2^16)
#define BUF_SZ 65536
//#define BUF_SZ 131072
#define DEFAULT_QUALITY 85

// Define the functions to use
void error_exit_compress(j_common_ptr cinfo);
void init_destination(j_compress_ptr cinfo);
void term_destination(j_compress_ptr cinfo);
boolean empty_output_buffer(j_compress_ptr cinfo);

// Define the metatable for JPEGs
#define MT_NAME "jpeg_mt"
typedef struct {
	j_common_ptr cinfo;
	JOCTET* buffer;
	size_t buffer_sz;
} structJPEG;
// Be able to check the input of a jpeg
static structJPEG * lua_checkjpeg(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid jpeg");
  return (structJPEG *)ud;
}


// TODO: There may be issues with free if this is called...
void error_exit_compress(j_common_ptr cinfo)
{
	fprintf(stdout,"Error exit!\n");
	fflush(stdout);
  (*cinfo->err->output_message) (cinfo);
  jpeg_destroy_compress((j_compress_ptr)cinfo);
}
void init_destination(j_compress_ptr cinfo) {
	fprintf(stdout,"Init destination!\n");
	fflush(stdout);
	structJPEG* ud = (structJPEG *)cinfo->client_data;
	ud->buffer = (JOCTET*)malloc(BUF_SZ);
	if(ud->buffer==NULL){
		fprintf(stdout,"Bad malloc!\n");
		fflush(stdout);
	}
	ud->buffer_sz = BUF_SZ;
  cinfo->dest->next_output_byte = ud->buffer;
  cinfo->dest->free_in_buffer = BUF_SZ;
	printf("nb: %x, %zu\n",cinfo->dest->next_output_byte,cinfo->dest->free_in_buffer);
}
void term_destination(j_compress_ptr cinfo) {
	fprintf(stdout,"Term destination!\n");
	fflush(stdout);
	
  /*
	cinfo->dest->next_output_byte = destBuf;
	cinfo->dest->free_in_buffer = destBufSize;
	*/
	structJPEG* ud = (structJPEG *)cinfo->client_data;
	uint8_t* destBuf = ud->buffer;
	
	// How much left
  int len = ud->buffer_sz - cinfo->dest->free_in_buffer;
	// TODO: Unsure what this does... (align?)
  while (len % 2 != 0)
    destBuf[len++] = 0xFF;

	// Reallocate
	if( (ud->buffer=realloc(ud->buffer, len))==NULL ){
		fprintf(stdout,"Bad realloc!");
		fflush(stdout);
	}
	ud->buffer_sz = len;
  //destBuf.resize(len);
}
boolean empty_output_buffer(j_compress_ptr cinfo) {
  fprintf(stdout,"Error buffer too small!\n");
	fflush(stdout);
	structJPEG* ud = (structJPEG *)cinfo->client_data;
	// TODO: Use realloc instead of the vector, for C only
	// Reallocate
	if( (ud->buffer=realloc(ud->buffer, ud->buffer_sz * 2))==NULL ){
		fprintf(stdout,"Bad realloc!");
		fflush(stdout);
	}
	cinfo->dest->free_in_buffer = ud->buffer_sz;
	cinfo->dest->next_output_byte = ud->buffer + ud->buffer_sz;
	ud->buffer_sz *= 2;
	/*
  unsigned int size = destBuf.size();
  destBuf.resize(2*size);
  cinfo->dest->next_output_byte = &(destBuf[size]);
  cinfo->dest->free_in_buffer = size;
	*/
	
  return TRUE;
}

/* Initialize a new compressor struct */
j_compress_ptr new_cinfo(structJPEG *ud){
	
	// Allocate on the heap
	// TODO: Garbage collection on the structJPEG metatable
	j_compress_ptr compress_info = (j_compress_ptr) malloc(sizeof(struct jpeg_compress_struct));
	j_common_ptr cinfo = (j_common_ptr)compress_info;
	
	// Setup error handling
	// TODO: Why malloc this?
	struct jpeg_error_mgr *jerr = (struct jpeg_error_mgr*) malloc(sizeof(struct jpeg_error_mgr));
  cinfo->err = jpeg_std_error(jerr);
  jerr->error_exit = error_exit_compress;
	
	// Initialize the struct for compression
	jpeg_create_compress(compress_info);
  if (compress_info->dest == NULL) {
		fprintf(stdout,"Make pool!\n");
		fflush(stdout);
    compress_info->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small)(cinfo, JPOOL_PERMANENT, sizeof(struct jpeg_destination_mgr));
		printf("dest: %x\n",compress_info->dest);
  }
  compress_info->dest->init_destination = init_destination;
  compress_info->dest->empty_output_buffer = empty_output_buffer;
  compress_info->dest->term_destination = term_destination;
	
	// set the client data for the compressor
	cinfo->client_data = ud;
	
	//cinfo->write_JFIF_header = TRUE;
  compress_info->write_JFIF_header = FALSE;
  //cinfo->dct_method = JDCT_IFAST;
  compress_info->dct_method = JDCT_FASTEST; // TurboJPEG
	
	jpeg_set_quality( compress_info, DEFAULT_QUALITY, TRUE);
	
	// NULL the buffer for use by jpeg
	ud->buffer = NULL;
	ud->cinfo = cinfo;
	
	return compress_info;
}

// Just compress a 2D array that is already in memory
static int lua_jpeg_compress(lua_State *L) {
	
	// JPEG struct with buffer and cinfo is our first
	structJPEG *ud = lua_checkjpeg(L, 1);
	
	// We will not modify the data
  const char* data;
	int width = luaL_checkint(L, 3);
  int height = luaL_checkint(L, 4);
	size_t n = width * height;
	size_t sz = 0;
	
	printf("w: %d, h: %d\n",width,height);
	
	// TODO: Check if a torch object
  if (lua_isstring(L, 2)) {
    data = lua_tolstring(L, 2, &sz);
		if(n != sz){
			return luaL_error(L, "Bad dimensions");
		}
  } else if (lua_islightuserdata(L, 2)) {
    data = (char*) lua_touserdata(L, 2);
  } else {
		return luaL_error(L, "Bad JPEG Compress 16 input");
  }
	
	j_compress_ptr cinfo = (j_compress_ptr) ud->cinfo;
	// Set the width and height for compression
  cinfo->image_width = width;
  cinfo->image_height = height;
	
	// Set the defaults based on the colorspace
	// http://refspecs.linuxbase.org/LSB_3.1.0/LSB-Desktop-generic/LSB-Desktop-generic/libjpeg.jpeg.set.defaults.1.html
	jpeg_set_defaults(cinfo);
	
	// Begin compression
	jpeg_start_compress(cinfo, TRUE);
	// Write all scanlines at once, from data
	size_t stride = cinfo->input_components * width;
	
	printf("HERE2\n");
	fflush(stdout);
	
	fprintf(stdout,"Data: %x",data);
	fflush(stdout);
	
  JSAMPROW row_pointer[1];
	JSAMPLE* img_ptr = (JSAMPLE*)data;	
	while (cinfo->next_scanline < height) {
		*row_pointer = img_ptr;
		jpeg_write_scanlines(cinfo, row_pointer, 1);
		img_ptr += stride;
	}
	
	// Check that the scanlines were written correctly
	//if(n_written!=cinfo->image_height){
	//	return luaL_error(L, "Did not write all scanlines!");
	//}
	// Finish the compression
  jpeg_finish_compress(cinfo);
	
	//int len = ud->buffer_sz - cinfo->dest->free_in_buffer;
	printf("len: %zu %zu\n",cinfo->dest->free_in_buffer,ud->buffer_sz);
	fflush(stdout);
	lua_pushlstring(L, (const char *)ud->buffer, ud->buffer_sz);

  //if (lenPacked > 0)
//    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  //else
  //  return luaL_error(L, "Compress Error");
	

  return 1;
}

static int lua_jpeg_rgb_new(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	
	// Form the compressor
	j_compress_ptr cinfo = new_cinfo(ud);
	
	// Grayscale specific
	cinfo->in_color_space = JCS_RGB;
	cinfo->input_components = 3;
	
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	
	// only pushed the metatable
	return 1;
}

static int lua_jpeg_yuv_new(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	
	// Form the compressor
	j_compress_ptr cinfo = new_cinfo(ud);
	
	// Grayscale specific
	cinfo->in_color_space = JCS_YCbCr;
	cinfo->input_components = 3;
	
	// only pushed the metatable
	return 1;
}

static int lua_jpeg_yuyv_new(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	
	// Form the compressor
	j_compress_ptr cinfo = new_cinfo(ud);
	
	// Grayscale specific
	cinfo->in_color_space = JCS_YCbCr;
	cinfo->input_components = 3;
	
	// only pushed the metatable
	return 1;
}

static int lua_jpeg_gray_new(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	
	// Form the compressor
	j_compress_ptr cinfo = new_cinfo(ud);
	
	// Grayscale specific
	cinfo->in_color_space = JCS_GRAYSCALE;
	cinfo->input_components = 1;
	
	// only pushed the metatable
	return 1;
}

/*
static int lua_decompress_new(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	
	// Form the compressor
	ud->cinfo = new_cinfo(ud);
	// set the client data for the compressor
	cinfo->client_data = ud;
	
	// only pushed the metatable
	return 1;
}
*/

static int lua_jpeg_quality(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  int quality = luaL_checkint(L, 2);
	if(quality<1 || quality>98)
  	return luaL_error(L, "Quality must be in range of 1 to 98");
	jpeg_set_quality( (j_compress_ptr)ud->cinfo, quality, TRUE);
  return 0;
}

static int lua_jpeg_delete(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  /* cleanup heap allocation */
	if(ud->buffer!=NULL){
  	free(ud->buffer);
	}
	
	if(ud->cinfo->err!=NULL){
		free(ud->cinfo->err);
	}
	
	if(ud->cinfo!=NULL){
		jpeg_destroy_compress((j_compress_ptr)ud->cinfo);
		free(ud->cinfo);
	}
	
  return 1;
}

static int lua_jpeg_tostring(lua_State *L) {
  structJPEG *p = lua_checkjpeg(L, 1);
  lua_pushstring(L,"Compressor");
  return 1;
}

static int lua_jpeg_index(lua_State *L) {
	if (!lua_getmetatable(L, 1)) {
		/* push metatable */
		lua_pop(L, 1); 
		return 0;
	}
	lua_pushvalue(L, 2); /* copy key */
	lua_rawget(L, -2); /* get metatable function */
	lua_remove(L, -2);  /* delete metatable */
	return 1;
}

static const struct luaL_reg jpeg_Methods [] = {
  {"compress", lua_jpeg_compress},
	{"quality", lua_jpeg_quality},
  {"__gc", lua_jpeg_delete},
	{"__tostring", lua_jpeg_tostring},
	{"__index", lua_jpeg_index},
  {NULL, NULL}
};

static const struct luaL_reg jpeg_Functions [] = {
  {"rgb", lua_jpeg_rgb_new},
	{"yuv", lua_jpeg_yuv_new},
  {"gray", lua_jpeg_gray_new},
  {"yuyv", lua_jpeg_yuyv_new},
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
