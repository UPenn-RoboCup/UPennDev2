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

//#define DEBUG 1
// UDP Friendly size (2^16)
//#define BUF_SZ 1024
#define BUF_SZ 65536
//#define BUF_SZ 131072
#define DEFAULT_QUALITY 85
#define USE_JFIF FALSE
// TurboJPEG: fastest
#define DEFAULT_DCT_METHOD JDCT_FASTEST
//#define DEFAULT_DCT_METHOD JDCT_IFAST

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
	uint8_t downsample; // Downsampling if needed
	uint8_t fmt;
} structJPEG;
// Be able to check the input of a jpeg
static structJPEG * lua_checkjpeg(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid jpeg");
  return (structJPEG *)ud;
}

static inline void fill_yuyv(JSAMPLE *img_row, JSAMPLE *buf_row, structJPEG* ud) {
	uint8_t Y0 = 0, Y1 = 0, U = 0, V = 0, downsample = ud->downsample;
	size_t row_idx = 0, width = ((j_compress_ptr)ud->cinfo)->image_width << downsample;
  while (row_idx < width) {
    Y0 = *img_row;
    U  = *(img_row++);
    Y1 = *(img_row++);
    V  = *(img_row++);
		// Always store the pixel
		*buf_row = Y0;
		*(buf_row++) = U;
		*(buf_row++) = V;
		// One pixel is written to buf_row
		row_idx++;
    if (downsample == 0) {
			// No downsampling, so we duplicate some data
			*buf_row = Y1;
			*(buf_row++) = U;
			*(buf_row++) = V;
			// Another pixel is written
			row_idx++;
    } else if (downsample == 2) {
			// Quarter size support, so skip a whole yuyv block
      img_row += 4;
    }
  }
}

// TODO: There may be issues with free if this is called...
void error_exit_compress(j_common_ptr cinfo) {
#ifdef DEBUG
	fprintf(stdout,"Error exit!\n");
	fflush(stdout);
#endif
  (*cinfo->err->output_message) (cinfo);
  jpeg_destroy_compress((j_compress_ptr)cinfo);
}
void init_destination(j_compress_ptr cinfo) {
#ifdef DEBUG
	fprintf(stdout,"Init destination!\n");
	fflush(stdout);
#endif
	structJPEG* ud = (structJPEG *)cinfo->client_data;
	
	if(ud->buffer==NULL){
		ud->buffer = (JOCTET*)malloc(BUF_SZ);
#ifdef DEBUG
		fprintf(stdout,"Bad malloc!\n");
		fflush(stdout);
#endif
	}
	ud->buffer_sz = BUF_SZ;
  cinfo->dest->next_output_byte = ud->buffer;
  cinfo->dest->free_in_buffer = BUF_SZ;
#ifdef DEBUG
	printf("nb: %x, %zu\n",cinfo->dest->next_output_byte,cinfo->dest->free_in_buffer);
#endif
}
void term_destination(j_compress_ptr cinfo) {
#ifdef DEBUG
	fprintf(stdout,"Term destination!\n");
	fflush(stdout);
#endif

	structJPEG* ud = (structJPEG *)cinfo->client_data;
	JOCTET* destBuf = ud->buffer;
	
	// How much left
  int len = ud->buffer_sz - cinfo->dest->free_in_buffer;
	// TODO: Unsure what this does... (align?)
  while (len % 2 != 0)
    destBuf[len++] = 0xFF;

	// Reallocate
	ud->buffer = realloc(ud->buffer, len);
	if(ud->buffer == NULL){
#ifdef DEBUG
		fprintf(stdout,"Bad realloc!\n");
		fflush(stdout);
#endif
		free(destBuf);
	} else {
		//free(destBuf);
	}
	
	ud->buffer_sz = len;
	
#ifdef DEBUG
	fprintf(stdout,"Done Term destination!\n");
	fflush(stdout);
#endif
	
}
boolean empty_output_buffer(j_compress_ptr cinfo) {
#ifdef DEBUG
  fprintf(stdout,"Error buffer too small!\n");
	fflush(stdout);
#endif
	structJPEG* ud = (structJPEG *)cinfo->client_data;
	JOCTET* destBuf = ud->buffer;
	// TODO: Use realloc instead of the vector, for C only
	// Reallocate
	ud->buffer = realloc(ud->buffer, ud->buffer_sz * 2);
	if(ud->buffer == NULL){
#ifdef DEBUG
		fprintf(stdout,"Bad realloc!\n");
		fflush(stdout);
#endif
		free(destBuf);
	} else {
		//free(destBuf);
	}
	
	cinfo->dest->free_in_buffer = ud->buffer_sz;
	cinfo->dest->next_output_byte = ud->buffer + ud->buffer_sz;
	ud->buffer_sz *= 2;
	
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
  cinfo->err->error_exit = error_exit_compress;
	
	// Initialize the struct for compression
	jpeg_create_compress(compress_info);
  if (compress_info->dest == NULL) {
#ifdef DEBUG
		fprintf(stdout,"Make pool!\n");
		fflush(stdout);
#endif
    compress_info->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small)(cinfo, JPOOL_PERMANENT, sizeof(struct jpeg_destination_mgr));
  }
#ifdef DEBUG
		printf("dest: %x\n",compress_info->dest);
#endif
  compress_info->dest->init_destination = init_destination;
  compress_info->dest->empty_output_buffer = empty_output_buffer;
  compress_info->dest->term_destination = term_destination;
	
	// set the client data for the compressor
	cinfo->client_data = ud;
	// Some compression properties
  compress_info->write_JFIF_header = USE_JFIF;
  compress_info->dct_method = DEFAULT_DCT_METHOD;
	
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
	
#ifdef DEBUG
	printf("w: %d, h: %d\n",width,height);
#endif
	
	// TODO: Check if a torch object
  if (lua_isstring(L, 2)) {
		size_t sz = 0;
    data = lua_tolstring(L, 2, &sz);
		/*
		size_t n = width * height;
		if(n != sz){
			return luaL_error(L, "Bad dimensions");
		}
		*/
  } else if (lua_islightuserdata(L, 2)) {
    data = (char*) lua_touserdata(L, 2);
  } else {
		return luaL_error(L, "Bad JPEG Compress 16 input");
  }
#ifdef DEBUG
	fprintf(stdout,"Data: %x\n",data);
	fflush(stdout);
#endif
	
	j_compress_ptr cinfo = (j_compress_ptr) ud->cinfo;
	// Set the width and height for compression
  cinfo->image_width  = width  >> (ud->downsample);
	cinfo->image_height = height >> (ud->downsample);
	
	// Set the defaults based on the colorspace
	// http://refspecs.linuxbase.org/LSB_3.1.0/LSB-Desktop-generic/LSB-Desktop-generic/libjpeg.jpeg.set.defaults.1.html
	jpeg_set_defaults(cinfo);
	
	JSAMPROW row_pointer[1];
	// Begin compression
	if( ud->fmt == 3 ){
		// Each row has this many bytes
		size_t stride = (width * 2) << (ud->downsample);
#ifdef DEBUG
		printf("stride: %d\n",stride);
		fflush(stdout);
		int line = 0;
#endif
		// Output image dimenstions
		// YUV row buffer
		JSAMPLE* row_buf = (JSAMPLE*)malloc( 3 * cinfo->image_width * sizeof(JSAMPLE) );
		JSAMPLE* img_ptr = (JSAMPLE*)data;
		jpeg_start_compress(cinfo, TRUE);
		while (cinfo->next_scanline < cinfo->image_height) {
#ifdef DEBUG
			printf("line: %d\n",line++);
			fflush(stdout);
#endif
			*row_pointer = img_ptr;
			// TODO: Maybe other fills for subsampling RGB or Grey
			fill_yuyv(img_ptr, row_buf, ud);
			jpeg_write_scanlines(cinfo, row_pointer, 1);
			img_ptr += stride;
		}
#ifdef DEBUG
			printf("free row_buf: %x\n",(void*)row_buf);
			fflush(stdout);
#endif
		free(row_buf);
	} else {
		// Default is just a raw copy
		// Suitable for jpeg and grayscale
		JSAMPLE* img_ptr = (JSAMPLE*)data;
		size_t stride = cinfo->input_components * width;
		jpeg_start_compress(cinfo, TRUE);
		while (cinfo->next_scanline < cinfo->image_height) {
			*row_pointer = img_ptr;
			jpeg_write_scanlines(cinfo, row_pointer, 1);
			img_ptr += stride;
		}
	}
	
		// Finish the compression
#ifdef DEBUG
		printf("finish compress1\n");
		fflush(stdout);
		printf("ud->buffer: %x\n",(void*)ud->buffer);
		fflush(stdout);
#endif
		jpeg_finish_compress(cinfo);
#ifdef DEBUG
			printf("finish compress2\n");
		fflush(stdout);
#endif

#ifdef DEBUG
	printf("len: %zu %zu\n",cinfo->dest->free_in_buffer,ud->buffer_sz);
	fflush(stdout);
#endif
	
	lua_pushlstring(L, (const char *)ud->buffer, ud->buffer_sz);	
  return 1;
}

static int lua_jpeg_new_compressor(lua_State *L) {
	// Form the struct of metadata
	// The function also pushes it to the stack
	structJPEG *ud = (structJPEG *)lua_newuserdata(L, sizeof(structJPEG));
	// Compressor format
	size_t len;
	const char * fmt = luaL_checklstring(L,1,&len);
	// Form the compressor
	j_compress_ptr cinfo = new_cinfo(ud);
	
	if (strncmp(fmt, "rgb", 3) == 0){
		cinfo->in_color_space = JCS_RGB;
		cinfo->input_components = 3;
		ud->downsample = 0;
		ud->fmt = 0;
	} else if (strncmp(fmt, "gray", 4) == 0){
		cinfo->in_color_space = JCS_GRAYSCALE;
		cinfo->input_components = 1;
		ud->downsample = 0;
		ud->fmt = 1;
	} else if (strncmp(fmt, "yuv", 4) == 0){
		cinfo->in_color_space = JCS_YCbCr;
		cinfo->input_components = 3;
		ud->downsample = 0;
		ud->fmt = 2;
	} else if (strncmp(fmt, "yuyv", 3) == 0){
		cinfo->in_color_space = JCS_YCbCr;
		cinfo->input_components = 3;
		ud->downsample = 0;
		ud->fmt = 3;
	} else {
		return luaL_error(L, "Unsupported format.");
	}
	
	// Set the metatable information
	luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
	// only pushed the metatable
	return 1;
}

static int lua_jpeg_quality(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  int quality = luaL_checkint(L, 2);
	if(quality<1 || quality>98)
  	return luaL_error(L, "Quality must be in range of 1 to 98");
	jpeg_set_quality( (j_compress_ptr)ud->cinfo, quality, TRUE);
  return 0;
}

static int lua_jpeg_downsampling(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  int dsample = luaL_checkint(L, 2);
	if(dsample<0 || dsample>4 || dsample==3)
  	return luaL_error(L, "Bad downsample factor");
	ud->downsample = dsample;
  return 0;
}

static int lua_jpeg_delete(lua_State *L) {
  structJPEG *ud = lua_checkjpeg(L, 1);
  /* cleanup heap allocations */
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
  structJPEG *ud = lua_checkjpeg(L, 1);
	j_compress_ptr cinfo = (j_compress_ptr)ud->cinfo;
	char buffer[64];
	switch(cinfo->in_color_space){
		case JCS_RGB:
			sprintf(buffer,"RGB: %d channels.",cinfo->input_components);
			break;
		case JCS_GRAYSCALE:
			sprintf(buffer,"Gray: %d channels.",cinfo->input_components);
			break;
		case JCS_YCbCr:
			sprintf(buffer,"YCbCr: %d channels.",cinfo->input_components);
			break;
		default:
			sprintf(buffer,"Unknown: %d channels.",cinfo->input_components);
	}
  lua_pushstring(L,buffer);
	
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
	{"downsampling", lua_jpeg_downsampling},
  {"__gc", lua_jpeg_delete},
	{"__tostring", lua_jpeg_tostring},
	{"__index", lua_jpeg_index},
  {NULL, NULL}
};

static const struct luaL_reg jpeg_Functions [] = {
	{"compressor", lua_jpeg_new_compressor},
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
