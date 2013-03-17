/*
 * Copyright 2002-2010 Guillaume Cottenceau.
 *
 * This software may be freely redistributed under the terms
 * of the X11 license.
 * 
 * lua wrapper by Yida Zhang <yida@seas.upenn.edu>
 * University of Pennsylvania
 */

#ifdef __cplusplus
extern "C"
{
#endif
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#ifdef __cplusplus
}
#endif

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define MT_NAME "cpng_mt"
#define PNG_DEBUG 3
#include <png.h>

void abort_(const char * s, ...)
{
  va_list args;
  va_start(args, s);
  vfprintf(stderr, s, args);
  fprintf(stderr, "\n");
  va_end(args);
  abort();
}


typedef struct {
  int width, height, stride;
  png_byte color_type;
  png_byte bit_depth;
  
  png_structp png_ptr;
  png_infop info_ptr;
  int number_of_passes;
  png_bytep * row_pointers;
} structPNG;

static structPNG * lua_checkcpng(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, MT_NAME);
  luaL_argcheck(L, *(structPNG **)ud != NULL, narg, "invalid png");
  return (structPNG *)ud;
}

static int lua_cpng_getValue(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index

  if ((index < 0) || (index >= p->height * p->stride)) {
    lua_pushnil(L);
    return 1;
  }
  
  int r = index / p->stride;
  int c = index - r * p->stride;
  
  lua_pushinteger(L, p->row_pointers[r][c]);

  return 1;
}

static int lua_cpng_delete(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  /* cleanup heap allocation */
  int y;
  for (y=0; y<p->height; y++)
          free(p->row_pointers[y]);
  free(p->row_pointers);

  return 1;
}

static int lua_cpng_setValue(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  int index = luaL_checkint(L, 2) - 1; // Convert lua 1-index to C 0-index

  if ((index < 0) || (index >= p->height * p->stride)) {
    lua_pushnil(L);
    return 1;
  }

  int val = lua_tointeger(L, 3);
  
  int r = index / p->stride;
  int c = index - r * p->stride;

  p->row_pointers[r][c] = val;
 
  return 1;
}

static int lua_cpng_len(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->height * p->stride);
  return 1;
}

static int lua_cpng_index(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  if ((lua_type(L, 2) == LUA_TNUMBER) && lua_tointeger(L, 2)) {
    // Numeric index:
    return lua_cpng_getValue(L);
  }

  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}


static int lua_cpng_new(lua_State *L) {
  structPNG *ud = (structPNG *)lua_newuserdata(L, sizeof(structPNG));

  const char* file_name = luaL_checkstring(L, 1);

  char header[8];    // 8 is the maximum size that can be checked

  /* open file and test for it being a png */
  FILE *fp = fopen(file_name, "rb");
  if (!fp)
          abort_("[read_png_file] File %s could not be opened for reading", file_name);
  fread(header, 1, 8, fp);
  if (png_sig_cmp((const png_byte*)header, 0, 8))
          abort_("[read_png_file] File %s is not recognized as a PNG file", file_name);

  /* initialize stuff */
  ud->png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!ud->png_ptr)
          abort_("[read_png_file] png_create_read_struct failed");

  ud->info_ptr = png_create_info_struct(ud->png_ptr);
  if (!ud->info_ptr)
          abort_("[read_png_file] png_create_info_struct failed");

  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[read_png_file] Error during init_io");

  png_init_io(ud->png_ptr, fp);
  png_set_sig_bytes(ud->png_ptr, 8);

  png_read_info(ud->png_ptr, ud->info_ptr);

  ud->width = png_get_image_width(ud->png_ptr, ud->info_ptr);
  ud->height = png_get_image_height(ud->png_ptr, ud->info_ptr);
  ud->color_type = png_get_color_type(ud->png_ptr, ud->info_ptr);
  ud->bit_depth = png_get_bit_depth(ud->png_ptr, ud->info_ptr);

  ud->number_of_passes = png_set_interlace_handling(ud->png_ptr);
  png_read_update_info(ud->png_ptr, ud->info_ptr);


  /* read file */
  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[read_png_file] Error during read_image");

  ud->row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * ud->height);

  int x, y;
  ud->stride = png_get_rowbytes(ud->png_ptr,ud->info_ptr);
  for (y=0; y<ud->height; y++)
    ud->row_pointers[y] = (png_byte*) malloc(png_get_rowbytes(ud->png_ptr,ud->info_ptr));

  png_read_image(ud->png_ptr, ud->row_pointers);

  fclose(fp);


  luaL_getmetatable(L, MT_NAME);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_cpng_read(lua_State *L) {
  return 1;
}

static int lua_cpng_write(lua_State *L) {
  structPNG *ud = lua_checkcpng(L, 1);
  const char* file_name = luaL_checkstring(L, 2);

  /* create file */
  FILE *fp = fopen(file_name, "wb");
  if (!fp)
          abort_("[write_png_file] File %s could not be opened for writing", file_name);


  /* initialize stuff */
  ud->png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!ud->png_ptr)
          abort_("[write_png_file] png_create_write_struct failed");

  ud->info_ptr = png_create_info_struct(ud->png_ptr);
  if (!ud->info_ptr)
          abort_("[write_png_file] png_create_info_struct failed");

  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[write_png_file] Error during init_io");

  png_init_io(ud->png_ptr, fp);


  /* write header */
  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[write_png_file] Error during writing header");

  png_set_IHDR(ud->png_ptr, ud->info_ptr, ud->width, ud->height,
               ud->bit_depth, ud->color_type, PNG_INTERLACE_NONE,
               PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

  png_write_info(ud->png_ptr, ud->info_ptr);


  /* write bytes */
  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[write_png_file] Error during writing bytes");

  png_write_image(ud->png_ptr, ud->row_pointers);


  /* end write */
  if (setjmp(png_jmpbuf(ud->png_ptr)))
          abort_("[write_png_file] Error during end of write");

  png_write_end(ud->png_ptr, NULL);

//  /* cleanup heap allocation */
//  int y;
//  for (y=0; y<ud->height; y++)
//          free(ud->row_pointers[y]);
//  free(row_pointers);

  fclose(fp);

  return 1;
}

static int lua_cpng_save(lua_State *L) {
  return 1;
}

//static int lua_cpng_read(lua_State *L)
//{
//  const char* file_name = luaL_checkstring(L, 1);
//
//  char header[8];    // 8 is the maximum size that can be checked
//
//  /* open file and test for it being a png */
//  FILE *fp = fopen(file_name, "rb");
//  if (!fp)
//          abort_("[read_png_file] File %s could not be opened for reading", file_name);
//  fread(header, 1, 8, fp);
//  if (png_sig_cmp((const png_byte*)header, 0, 8))
//          abort_("[read_png_file] File %s is not recognized as a PNG file", file_name);
//
//  /* initialize stuff */
//  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
//
//  if (!png_ptr)
//          abort_("[read_png_file] png_create_read_struct failed");
//
//  info_ptr = png_create_info_struct(png_ptr);
//  if (!info_ptr)
//          abort_("[read_png_file] png_create_info_struct failed");
//
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[read_png_file] Error during init_io");
//
//  png_init_io(png_ptr, fp);
//  png_set_sig_bytes(png_ptr, 8);
//
//  png_read_info(png_ptr, info_ptr);
//
//  width = png_get_image_width(png_ptr, info_ptr);
//  height = png_get_image_height(png_ptr, info_ptr);
//  color_type = png_get_color_type(png_ptr, info_ptr);
//  bit_depth = png_get_bit_depth(png_ptr, info_ptr);
//
//  number_of_passes = png_set_interlace_handling(png_ptr);
//  png_read_update_info(png_ptr, info_ptr);
//
//
//  /* read file */
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[read_png_file] Error during read_image");
//
//  row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
//  for (y=0; y<height; y++)
//          row_pointers[y] = (png_byte*) malloc(png_get_rowbytes(png_ptr,info_ptr));
//
//  png_read_image(png_ptr, row_pointers);
//
//  fclose(fp);
//  return 1;
//}
//
//static int lua_cpng_write(lua_State *L)
//{
//  const char* file_name = luaL_checkstring(L, 1);
//
//  /* create file */
//  FILE *fp = fopen(file_name, "wb");
//  if (!fp)
//          abort_("[write_png_file] File %s could not be opened for writing", file_name);
//
//
//  /* initialize stuff */
//  png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
//
//  if (!png_ptr)
//          abort_("[write_png_file] png_create_write_struct failed");
//
//  info_ptr = png_create_info_struct(png_ptr);
//  if (!info_ptr)
//          abort_("[write_png_file] png_create_info_struct failed");
//
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[write_png_file] Error during init_io");
//
//  png_init_io(png_ptr, fp);
//
//
//  /* write header */
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[write_png_file] Error during writing header");
//
//  png_set_IHDR(png_ptr, info_ptr, width, height,
//               bit_depth, color_type, PNG_INTERLACE_NONE,
//               PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
//
//  png_write_info(png_ptr, info_ptr);
//
//
//  /* write bytes */
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[write_png_file] Error during writing bytes");
//
//  png_write_image(png_ptr, row_pointers);
//
//
//  /* end write */
//  if (setjmp(png_jmpbuf(png_ptr)))
//          abort_("[write_png_file] Error during end of write");
//
//  png_write_end(png_ptr, NULL);
//
//  /* cleanup heap allocation */
//  for (y=0; y<height; y++)
//          free(row_pointers[y]);
//  free(row_pointers);
//
//  fclose(fp);
//  return 1;
//}


//void process_file(void)
//{
//        if (png_get_color_type(png_ptr, info_ptr) == PNG_COLOR_TYPE_RGB)
//                abort_("[process_file] input file is PNG_COLOR_TYPE_RGB but must be PNG_COLOR_TYPE_RGBA "
//                       "(lacks the alpha channel)");
//
//        if (png_get_color_type(png_ptr, info_ptr) != PNG_COLOR_TYPE_RGBA)
//                abort_("[process_file] color_type of input file must be PNG_COLOR_TYPE_RGBA (%d) (is %d)",
//                       PNG_COLOR_TYPE_RGBA, png_get_color_type(png_ptr, info_ptr));
//
//        for (y=0; y<height; y++) {
//                png_byte* row = row_pointers[y];
//                for (x=0; x<width; x++) {
//                        png_byte* ptr = &(row[x*4]);
//                        printf("Pixel at position [ %d - %d ] has RGBA values: %d - %d - %d - %d\n",
//                               x, y, ptr[0], ptr[1], ptr[2], ptr[3]);
//
//                        /* set red value to 0 and green value to the blue one */
//                        ptr[0] = 0;
//                        ptr[1] = ptr[2];
//                }
//        }
//}


//int main(int argc, char **argv)
//{
//        if (argc != 3)
//                abort_("Usage: program_name <file_in> <file_out>");
//
//        read_png_file(argv[1]);
////        process_file();
//        write_png_file(argv[2]);
//
//        return 0;
//}

static int lua_cpng_pointer(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
//  int offset = luaL_optint(L, 2, 0);

//  lua_pushlightuserdata(L, ((char *)p->ptr + offset));
  return 1;
}

static int lua_cpng_width(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->width);
  return 1;
}

static int lua_cpng_height(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->height);
  return 1;
}

static int lua_cpng_color_type(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->color_type);
  return 1;
}

static int lua_cpng_bit_depth(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->bit_depth);
  return 1;
}

static int lua_cpng_stride(lua_State *L) {
  structPNG *p = lua_checkcpng(L, 1);
  lua_pushinteger(L, p->stride);
  return 1;
}

static const struct luaL_reg cpng_Functions [] = {
  {"new", lua_cpng_new},
  {"save", lua_cpng_save},
  {NULL, NULL}
};

static const struct luaL_reg cpng_Methods [] = {
  {"pointer", lua_cpng_pointer},
  {"read", lua_cpng_read},
  {"write", lua_cpng_write},
  {"width", lua_cpng_width},
  {"height", lua_cpng_height},
  {"stride", lua_cpng_stride},
  {"color_type", lua_cpng_color_type},
  {"bit_depth", lua_cpng_bit_depth},
  {"__gc", lua_cpng_delete},
  {"__newindex", lua_cpng_setValue},
  {"__len", lua_cpng_len},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_cpng(lua_State *L) {
  luaL_newmetatable(L, MT_NAME);

  // Implement index method:
  lua_pushstring(L, "__index");
  lua_pushcfunction(L, lua_cpng_index);
  lua_settable(L, -3);

  luaL_register(L, NULL, cpng_Methods);
  luaL_register(L, "cpng", cpng_Functions);

  luaL_register(L, NULL, cpng_Methods);
  luaL_register(L, "cpng", cpng_Functions);
  return 1;
}
