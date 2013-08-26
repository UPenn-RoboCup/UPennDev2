#ifndef __JPEG_UTILS_H__
#define __JPEG_UTILS_H__

#include <string>
#include <vector>
#include <stdint.h>
#include <stdlib.h>
//#include <stdio.h>
#include <setjmp.h>
#include <jpeglib.h>

typedef struct {
  int width;
  int height;
  int stride;
  unsigned char *raw_image;
} structJPEG;

struct mem_jpeg_source_mgr : public jpeg_source_mgr
{
  bool buffer_filled;
  char dummy_buffer[ 2 ];

  mem_jpeg_source_mgr( char* data, unsigned int length );
};

struct mem_error_mgr : public jpeg_error_mgr 
{
  jmp_buf setjmp_buffer;
};


void error_exit_compress(j_common_ptr cinfo);
void init_destination(j_compress_ptr cinfo);
boolean empty_output_buffer(j_compress_ptr cinfo);
void term_destination(j_compress_ptr cinfo);
void mem_error_exit( j_common_ptr cinfo );
void mem_error_exit( j_common_ptr cinfo );
void jpeg_init_source( j_decompress_ptr cinfo );
boolean jpeg_fill_input_buffer( j_decompress_ptr cinfo );
void jpeg_skip_input_data( j_decompress_ptr cinfo, long num_bytes );
void jpeg_term_source (j_decompress_ptr cinfo);

#endif
