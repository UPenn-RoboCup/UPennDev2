/* 
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

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

#include <stdint.h>
#include <vector>
#include <string>
#include <jpeglib.h>

std::vector<unsigned char> destBuf;

static void error_exit(j_common_ptr cinfo)
{
  (*cinfo->err->output_message) (cinfo);
  jpeg_destroy_compress((j_compress_ptr) cinfo);
}

void init_destination(j_compress_ptr cinfo) {
//  const unsigned int size = 65536; //UDP friendly size
  const unsigned int size = 2*65536;
  destBuf.resize(size);
  cinfo->dest->next_output_byte = &(destBuf[0]);
  cinfo->dest->free_in_buffer = size;
}

boolean empty_output_buffer(j_compress_ptr cinfo)
{

  fprintf(stdout,"Error buffer too small!\n");

  unsigned int size = destBuf.size();
  destBuf.resize(2*size);
  cinfo->dest->next_output_byte = &(destBuf[size]);
  cinfo->dest->free_in_buffer = size;

  return TRUE;
}

void term_destination(j_compress_ptr cinfo) {
  /*
     cinfo->dest->next_output_byte = destBuf;
     cinfo->dest->free_in_buffer = destBufSize;
     */
  int len = destBuf.size() - (cinfo->dest->free_in_buffer);
  while (len % 2 != 0)
    destBuf[len++] = 0xFF;

  destBuf.resize(len);
}
int CompressData(const uint8_t* prRGB, int width, int height, int ch) {

  //fprintf(stdout,"compressing %dx%d\n",width, height);

  int quality = 90;

  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jerr.error_exit = error_exit;

  jpeg_create_compress(&cinfo);
  if (cinfo.dest == NULL) {
    cinfo.dest = (struct jpeg_destination_mgr *)
      (*cinfo.mem->alloc_small) ((j_common_ptr) &cinfo, JPOOL_PERMANENT,
          sizeof(struct jpeg_destination_mgr));
  }
  cinfo.dest->init_destination = init_destination;
  cinfo.dest->empty_output_buffer = empty_output_buffer;
  cinfo.dest->term_destination = term_destination;

  cinfo.image_width = width;
  cinfo.image_height = height;
  cinfo.input_components = 3;
  cinfo.in_color_space = JCS_RGB;
  if(ch==1){
    cinfo.input_components = 1;
    cinfo.in_color_space = JCS_GRAYSCALE;
  }
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  cinfo.write_JFIF_header = false;
  cinfo.dct_method = JDCT_IFAST;
  //cinfo.dct_method = JDCT_FASTEST; // TurboJPEG

  jpeg_start_compress(&cinfo, TRUE);

  /*
  JSAMPROW row_pointer;
  while (cinfo.next_scanline < cinfo.image_height) {
    row_pointer = (JSAMPROW) &prRGB[cinfo.next_scanline*width*3];
    jpeg_write_scanlines(&cinfo, &row_pointer, 1);
    fprintf(stdout,"RGB: %d,%d,%d\t",
        row_pointer[0],row_pointer[1],row_pointer[2]);
    fprintf(stdout,"RGB: %d,%d,%d\n",
        row_pointer[3],row_pointer[4],row_pointer[5]);
  }
  */
  JSAMPLE row[ch*width];
  JSAMPROW row_pointer[1];
  *row_pointer = row;
  while (cinfo.next_scanline < cinfo.image_height) {
    //fprintf(stdout,"cinfo.next_scanline = %d\n", cinfo.next_scanline);
    const uint8_t *p = prRGB + ch*width*cinfo.next_scanline;
    int irow = 0;
    for (int i = 0; i < width; i++) {
      if( ch==3 ){
        row[irow++] = *(p+i*ch);
        row[irow++] = *(p+i*ch+1);
        row[irow++] = *(p+i*ch+2);
      } else if( ch==4) {
        row[irow++] = *(p+i*ch+2);
        row[irow++] = *(p+i*ch+1);
        row[irow++] = *(p+i*ch);
      } else {
        row[irow++] = *(p+i);
      }
      //irow++;
  /*
      fprintf(stdout,"RGB: %d,%d,%d\t",
        row[0],row[1],row[2]);
    fprintf(stdout,"RGB: %d,%d,%d\n",
        row[3],row[4],row[5]);
    */
    }
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);

  unsigned int destBufSize = destBuf.size();
 /* 
  plhs[0] = mxCreateNumericMatrix(1, destBufSize, mxUINT8_CLASS, mxREAL);
  memcpy(mxGetData(plhs[0]), &(destBuf[0]), destBufSize);
*/
  return destBufSize;		
}

static int lua_cjpeg(lua_State *L) {
  uint8_t * dataSrc = (uint8_t *) lua_touserdata(L, 1);
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  int byte_sized = luaL_optint(L, 4, 3);
  int lenPacked = CompressData( dataSrc, width, height, byte_sized );

  if (lenPacked > 0)
    lua_pushlstring(L, (const char *)&(destBuf[0]), lenPacked);
  else
    return luaL_error(L, "Compress Error");

  return 1;
}


static const struct luaL_reg cjpeg_lib [] = {
  {"compress", lua_cjpeg},
  //      {"uncompress", lua_z_uncompress},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_cjpeg (lua_State *L) {
  luaL_register(L, "cjpeg", cjpeg_lib);
  return 1;
}
