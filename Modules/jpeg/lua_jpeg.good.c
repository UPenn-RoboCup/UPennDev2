/* 
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

/* TODO
 * Add torch support, then we can compress a sub window of the camera image
 If we can add torch support to the uvc library - 
 should be a simple thing with pointers
*/

#include <lua.h>
#include <lauxlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "jpeglib.h"

#ifdef TORCH
#include <torch/luaT.h>
#include <torch/TH/TH.h>
#endif


/* UDP Friendly size (2^16) */
#define BUF_SZ 65536
//#define BUF_SZ 131072
#define DEFAULT_QUALITY 85
#define USE_JFIF FALSE
// TurboJPEG: fastest
#define DEFAULT_DCT_METHOD JDCT_FASTEST
//#define DEFAULT_DCT_METHOD JDCT_IFAST

/* Define the functions to use */
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
	uint8_t subsample; // Downsampling if needed
	uint8_t fmt;
} structJPEG;
// Be able to check the input of a jpeg
static structJPEG * lua_checkjpeg(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, ud != NULL, narg, "invalid jpeg");
  return (structJPEG *)ud;
}

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
		fprintf(stdout,"Malloc! %p\n",(void*)ud->buffer);
		fflush(stdout);
#endif
	}
	ud->buffer_sz = BUF_SZ;
  cinfo->dest->next_output_byte = ud->buffer;
  cinfo->dest->free_in_buffer = BUF_SZ;
#ifdef DEBUG
	printf("nb: %p, %zu\n", (void*)cinfo->dest->next_output_byte,cinfo->dest->free_in_buffer);
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

	ud->buffer_sz = len;
	// Reallocate is unnecessary to smaller size
	/*
	ud->buffer = realloc(ud->buffer, len);
	if(ud->buffer == NULL){
#ifdef DEBUG
		fprintf(stdout,"Bad realloc!\n");
		fflush(stdout);
#endif
		free(destBuf);
	}
	*/
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
	size_t new_size = ud->buffer_sz * 2;
	ud->buffer = realloc(ud->buffer, new_size);
	if(ud->buffer == NULL){
#ifdef DEBUG
		fprintf(stdout,"Bad realloc!\n");
		fflush(stdout);
#endif
		free(destBuf);
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
		printf("dest: %p\n",(void*)compress_info->dest);
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
// Crop the image, though
static int lua_jpeg_compress_crop(lua_State *L) {
	
	JDIMENSION width, height, w0, w_cropped, h0, h_cropped;
  JSAMPLE* data;
  size_t stride;
	// JPEG struct with buffer and cinfo is our first
	structJPEG *ud = lua_checkjpeg(L, 1);
  
	// Access the JPEG compression settings
	j_compress_ptr cinfo = (j_compress_ptr) ud->cinfo;
	
	// TODO: Check if a torch object
  if (lua_isstring(L, 2)) {
		size_t sz = 0;
    data = (JSAMPLE*) lua_tolstring(L, 2, &sz);
  	width  = luaL_checkint(L, 3);
  	height = luaL_checkint(L, 4);
		stride = cinfo->input_components * width;
  } else if (lua_islightuserdata(L, 2)) {
    data = (JSAMPLE*) lua_touserdata(L, 2);
  	width  = luaL_checkint(L, 3);
  	height = luaL_checkint(L, 4);
		stride = cinfo->input_components * width;
  }
#ifdef TORCH
	else {
		THByteTensor* b_t = 
			(THByteTensor *) luaT_checkudata(L, 2, luaT_typename(L, 2));
		// TODO: Check continguous (or add stride support)
		data = b_t->storage->data;
		// Use the torch dimensions
		width = b_t->size[1];
		height = b_t->size[0];
    // Must override for crop, since easier :)
  	width  = luaL_checkint(L, 3);
  	height = luaL_checkint(L, 4);
		// TODO: Double check :)
		stride = b_t->stride[0];
	}
#else
	else {
		return luaL_error(L, "Bad JPEG Compress 16 input");
  }
#endif
	

	// Start and stop for crop (put into C indices)
	w0  = luaL_optint(L, 5, 1) - 1;
  w_cropped  = luaL_checkint(L, 6);
	h0  = luaL_optint(L, 7, 1) - 1;
  h_cropped  = luaL_checkint(L, 8);
	
	if(h_cropped+h0>height||w_cropped+w0>width){
		return luaL_error(L,"Bad crop");
	}
	
#ifdef DEBUG
	printf("w: %d, h: %d | %d %d\n",width,height,cinfo->image_width,cinfo->image_height);
	fprintf(stdout,"Data: %p\n",(void*)data);
	fflush(stdout);
#endif
	
	// Colorspace of the OUTPUTTED jpeg same is input (save space/speed?)
	// YCbCr and Grayscale are JFIF. RGB and others are Adobe
	jpeg_set_colorspace(cinfo, cinfo->in_color_space);
	
	// Copy the reference to data for pointer arithmetic
	JDIMENSION nlines;
	JSAMPROW row_pointer[1];
	JSAMPLE* img_ptr = data;
	
	size_t remainder = 0;
	if(ud->fmt==3){
		// Safe cropping for YUYV (align to a pixel)
		if (ud->subsample==2){
			w0 -= (w0%4);
			w_cropped -= (w_cropped%4);
		} else {
			w0 -= w0%2;
			w_cropped -= (w_cropped%2);
		}
		// YUYV is 2 bytes per pix
		img_ptr += 2 * h0 * width + w0;
		remainder = 2 * (width - w_cropped);
	} else {
		img_ptr += cinfo->input_components * (width * h0 + w0);
	}
	
	// Set the width and height for compression
  cinfo->image_width  = w_cropped >> ud->subsample;
	cinfo->image_height = h_cropped >> ud->subsample;
	int w = cinfo->image_width;
	int h = cinfo->image_height;
	
#ifdef DEBUG
	int line = 0;
	printf("remainder %zu",remainder);
#endif
	
	// Begin compression
	jpeg_start_compress(cinfo, TRUE);
	
	// YUYV is special
	if(ud->fmt==3){
		size_t img_stride = 2 * width * ud->subsample;
		img_stride += remainder;
		JSAMPLE* yuv_row = (JSAMPLE*)malloc( 3 * w * sizeof(JSAMPLE) );
		if(yuv_row==NULL){
			return luaL_error(L,"Bad malloc of yuv_row");
		}
		*row_pointer = yuv_row;
		int i;
		uint8_t y0,u,y1,v;
		while (cinfo->next_scanline < h) {
			JSAMPLE* yuv_pixel = yuv_row;
#ifdef DEBUG
			line++;
#endif
			i=0;
			while(i<w){
				y0 = *img_ptr++;
				u  = *img_ptr++;
				y1 = *img_ptr++;
				v  = *img_ptr++;
				if (ud->subsample==2){
					// Skip the next pixel, too
					img_ptr += 4;
				}
				//
				*yuv_pixel = y0;
				yuv_pixel++;
				*yuv_pixel = u;
				yuv_pixel++;
				*yuv_pixel = v;
				yuv_pixel++;
				if (ud->subsample) {
					// If subsampling, then we add only one pixel
					i++;
				} else {
					// Ignore this pixel if subsampling
					*yuv_pixel = y1;
					yuv_pixel++;
					*yuv_pixel = u;
					yuv_pixel++;
					*yuv_pixel = v;
					yuv_pixel++;
					// 2 pixels
					i+=2;
				}
				
#ifdef DEBUG
/*
				if(line<2){
					printf("line: (%d,%d): Y: %d, U: %d, Y: %d, V: %d, %d\n",line, i,y0,u,y1,v,yuv_row[0]);
				}
*/
#endif
			}
			nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
			if (ud->subsample) {
				// Skip a row if this subsample level
				img_ptr += img_stride;
			}
		}
		free(yuv_row);
	} else {
		while (cinfo->next_scanline < h) {
			*row_pointer = img_ptr;
			nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
			img_ptr += stride;
		}
	}

#ifdef DEBUG
	printf("len: %zu %zu %p\n",cinfo->dest->free_in_buffer,ud->buffer_sz,ud->buffer);
	fflush(stdout);
#endif
	
	jpeg_finish_compress(cinfo);
	lua_pushlstring(L, (const char*)ud->buffer, ud->buffer_sz);
  return 1;
}

// Just compress a 2D array that is already in memory
static int lua_jpeg_compress(lua_State *L) {
	
	// JPEG struct with buffer and cinfo is our first
	structJPEG *ud = lua_checkjpeg(L, 1);
	// Access the JPEG compression settings
	j_compress_ptr cinfo = (j_compress_ptr) ud->cinfo;
	
	// We will not modify the data
	// Use the JPEG typedef notations
  JSAMPLE* data;
	JDIMENSION width, height;
	size_t stride;

	// TODO: Check if a torch object
  if (lua_isstring(L, 2)) {
		size_t sz = 0;
    data = (JSAMPLE*) lua_tolstring(L, 2, &sz);
		width  = luaL_checkint(L, 3);
		height = luaL_checkint(L, 4);
		stride = cinfo->input_components * width;
	} else if (lua_islightuserdata(L, 2)) {
    data = (JSAMPLE*) lua_touserdata(L, 2);
		width  = luaL_checkint(L, 3);
		height = luaL_checkint(L, 4);
		stride = cinfo->input_components * width;
  }
	else if( lua_type(L, 2) == LUA_TNUMBER ){
		data = (uint8_t *)luaL_optlong(L, 2, 0);
    if (data == NULL) {
      return luaL_error(L, "Input image bad");
    }
		width  = luaL_checkint(L, 3);
		height = luaL_checkint(L, 4);
    stride = cinfo->input_components * width;
	}
#ifdef TORCH
	else {
		THByteTensor* b_t = 
			(THByteTensor *) luaT_checkudata(L, 2, luaT_typename(L, 2));
		// TODO: Check continguous (or add stride support)
		data = b_t->storage->data;
		// Use the torch dimensions
		width = b_t->size[1];
		height = b_t->size[0];
    // Can override
		width  = luaL_optint(L, 3, width);
		height = luaL_optint(L, 4, height);
		// TODO: Double check :)
		stride = b_t->stride[0];
	}
#else
	else {
		return luaL_error(L, "Bad JPEG Compress 16 input");
  }
#endif
	
	// Set the width and height for compression
  cinfo->image_width  = width  >> ud->subsample;
	cinfo->image_height = height >> ud->subsample;
	
#ifdef DEBUG
	printf("w: %d, h: %d | %d %d\n",width,height,cinfo->image_width,cinfo->image_height);
	fprintf(stdout,"Data: %p\n",(void*)data);
	fflush(stdout);
#endif
	
	// Colorspace of the OUTPUTTED jpeg same is input (save space/speed?)
	// YCbCr and Grayscale are JFIF. RGB and others are Adobe
	jpeg_set_colorspace(cinfo, cinfo->in_color_space);
	
	// Copy the reference to data for pointer arithmetic
	JDIMENSION nlines;
	JSAMPROW row_pointer[1];
	JSAMPLE* img_ptr = data;
	
#ifdef DEBUG
		int line = 0;
#endif
	
	// Begin compression
	jpeg_start_compress(cinfo, TRUE);
	
	// YUYV is special
	if(ud->fmt==3){
		int w = cinfo->image_width;
		int h = cinfo->image_height;
		size_t img_stride = 2 * width * ud->subsample;
		JSAMPLE* yuv_row = (JSAMPLE*)malloc( 3 * w * sizeof(JSAMPLE) );
		if(yuv_row==NULL){
			return luaL_error(L,"Bad malloc of yuv_row");
		}
		*row_pointer = yuv_row;
		int i;
		uint8_t y0,u,y1,v;
		while (cinfo->next_scanline < h) {
			JSAMPLE* yuv_pixel = yuv_row;
#ifdef DEBUG
			line++;
#endif
			i=0;
			while(i<w){
				y0 = *img_ptr++;
				u  = *img_ptr++;
				y1 = *img_ptr++;
				v  = *img_ptr++;
				if (ud->subsample==2){
					// Skip the next pixel, too
					img_ptr += 4;
				}
				//
				*yuv_pixel = y0;
				yuv_pixel++;
				*yuv_pixel = u;
				yuv_pixel++;
				*yuv_pixel = v;
				yuv_pixel++;
				if (ud->subsample) {
					// If subsampling, then we add only one pixel
					i++;
				} else {
					// Ignore this pixel if subsampling
					*yuv_pixel = y1;
					yuv_pixel++;
					*yuv_pixel = u;
					yuv_pixel++;
					*yuv_pixel = v;
					yuv_pixel++;
					// 2 pixels
					i+=2;
				}
				
#ifdef DEBUG
				/*
				if(line<5){
					printf("line: (%d,%d): Y: %d, U: %d, Y: %d, V: %d, %d\n",line, i,y0,u,y1,v,yuv_row[0]);
				}
				*/
#endif
			}
			nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
			if (ud->subsample) {
				// Skip a row if this subsample level
				img_ptr += img_stride;
			}
		}
		free(yuv_row);
	} else {
		int h = cinfo->image_height;
		while (cinfo->next_scanline < h) {
			*row_pointer = img_ptr;
			nlines = jpeg_write_scanlines(cinfo, row_pointer, 1);
			img_ptr += stride;
		}
	}

#ifdef DEBUG
	printf("len: %zu %zu %p\n",cinfo->dest->free_in_buffer,ud->buffer_sz,ud->buffer);
	fflush(stdout);
#endif
	
	jpeg_finish_compress(cinfo);
	lua_pushlstring(L, (const char*)ud->buffer, ud->buffer_sz);
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
	ud->subsample = 0;
	
	if (strncmp(fmt, "rgb", 3) == 0){
		cinfo->in_color_space = JCS_RGB;
		cinfo->input_components = 3;
		ud->fmt = 0;
	} else if (strncmp(fmt, "gray", 4) == 0){
		cinfo->in_color_space = JCS_GRAYSCALE;
		cinfo->input_components = 1;
		ud->fmt = 1;
	} else if (strncmp(fmt, "yuv", 4) == 0){
		cinfo->in_color_space = JCS_YCbCr;
		cinfo->input_components = 3;
		ud->fmt = 2;
	} else if (strncmp(fmt, "yuyv", 3) == 0){
		cinfo->in_color_space = JCS_YCbCr;
		cinfo->input_components = 3;
		ud->fmt = 3;
	} else {
		return luaL_error(L, "Unsupported format.");
	}
	
	// Set the defaults based on the colorspace
	// http://refspecs.linuxbase.org/LSB_3.1.0/LSB-Desktop-generic/LSB-Desktop-generic/libjpeg.jpeg.set.defaults.1.html
	jpeg_set_defaults(cinfo);
	
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
	if(ud->fmt!=3){
		return luaL_error(L, "Only YUYV subsampling!");
	}
  int dsample = luaL_checkint(L, 2);
	if(dsample<0 || dsample>2){
  	return luaL_error(L, "Bad subsample factor!");
	}
	ud->subsample = dsample;
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

static const struct luaL_Reg jpeg_Methods [] = {
  {"compress", lua_jpeg_compress},
	{"compress_crop", lua_jpeg_compress_crop},
	{"quality", lua_jpeg_quality},
	{"downsampling", lua_jpeg_downsampling},
  {"__gc", lua_jpeg_delete},
	{"__tostring", lua_jpeg_tostring},
	{"__index", lua_jpeg_index},
  {NULL, NULL}
};

static const struct luaL_Reg jpeg_Functions [] = {
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
