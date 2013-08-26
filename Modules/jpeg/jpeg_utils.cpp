/* 
 * (c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
 * ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
 * University of Pennsylvania
 * */

#include "jpeg_utils.h"
#include <stdio.h>

extern std::vector<unsigned char> destBuf;

void error_exit_compress(j_common_ptr cinfo)
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

void mem_error_exit( j_common_ptr cinfo )
{
  mem_error_mgr* err = (mem_error_mgr*) cinfo->err;
  longjmp( err->setjmp_buffer, 1 );
}

void jpeg_init_source( j_decompress_ptr cinfo )
{
}

boolean jpeg_fill_input_buffer( j_decompress_ptr cinfo )
{
  mem_jpeg_source_mgr* src = (mem_jpeg_source_mgr*)cinfo->src;
  if( src->buffer_filled ) 
  {
    // Insert a fake EOI marker - as per jpeglib recommendation
    src->next_input_byte = (const JOCTET*) src->dummy_buffer;
    src->bytes_in_buffer = 2;
  } 
  else 
  {
    src->buffer_filled = true;
  }
  return true;
}

void jpeg_skip_input_data( j_decompress_ptr cinfo, long num_bytes )
{
  mem_jpeg_source_mgr* src = (mem_jpeg_source_mgr*)cinfo->src;

  if (num_bytes > 0) 
  {
    while( num_bytes > (long) src->bytes_in_buffer ) 
    {
      num_bytes -= (long) src->bytes_in_buffer;
      jpeg_fill_input_buffer(cinfo);
    }
    src->next_input_byte += (size_t) num_bytes;
    src->bytes_in_buffer -= (size_t) num_bytes;
  }
}

void jpeg_term_source (j_decompress_ptr cinfo)
{
  /* no work necessary here */
}

mem_jpeg_source_mgr::mem_jpeg_source_mgr( char* data, unsigned int length )
: buffer_filled( false )
{
  dummy_buffer[ 0 ] = (JOCTET) 0xFF;
  dummy_buffer[ 1 ] = (JOCTET) JPEG_EOI;
  next_input_byte = (const JOCTET*) data;
  bytes_in_buffer = length;
  init_source = jpeg_init_source;
  fill_input_buffer = jpeg_fill_input_buffer;
  skip_input_data = jpeg_skip_input_data;
  resync_to_restart = jpeg_resync_to_restart;
  term_source = jpeg_term_source;
}

