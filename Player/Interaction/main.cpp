/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <OpenNI.h>

#include "../Common/OniSampleUtilities.h"

#include <zmq.h>



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
//  const unsigned int size = 65536;
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
int CompressData(const uint8_t* prRGB, int width, int height) {

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
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, quality, TRUE);
  cinfo.write_JFIF_header = true;//false;
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
  JSAMPLE row[3*width];
  JSAMPROW row_pointer[1];
  *row_pointer = row;
#ifdef WEBOTS
#define BGR
  const uint8_t ch = 4; //nchannels for RGBA space
#else
  const uint8_t ch = 3; //nchannels
#endif
  while (cinfo.next_scanline < cinfo.image_height) {
    //fprintf(stdout,"cinfo.next_scanline = %d\n", cinfo.next_scanline);
    const uint8_t *p = prRGB + ch*width*cinfo.next_scanline;
    int irow = 0;
    for (int i = 0; i < width; i++) {
#define BGR
#ifdef BGR
      row[irow++] = *(p+i*ch+2);
      row[irow++] = *(p+i*ch+1);
      row[irow++] = *(p+i*ch);
#else
      row[irow++] = *(p+i*ch);
      row[irow++] = *(p+i*ch+1);
      row[irow++] = *(p+i*ch+2);
#endif
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





using namespace openni;

int main()
{
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth, color;

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
    rc = depth.start();
    if (rc != STATUS_OK)
    {
      printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
      return 4;
    }
  }


  if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
    rc = color.create(device, SENSOR_COLOR);
    if (rc == STATUS_OK)
    {
      rc = color.start();
      if (rc != STATUS_OK)
      {   
        printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
        color.destroy();
        return 4;
      }   
    }
    else
    {
      printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
      return 3;
    }
  }


  // Set up ZMQ
  void *ctx, *socket;
  int result, rc2;
  ctx = zmq_init (2); //2 threads
  assert(ctx);
  socket = zmq_socket (ctx, ZMQ_PUB);
  assert( socket );
  rc2 = zmq_bind( socket, "ipc:///tmp/prime" );
  assert( rc2==0 );

  void* csocket;
  csocket = zmq_socket (ctx, ZMQ_PUB);
  assert( csocket );
  rc2 = zmq_bind( csocket, "ipc:///tmp/img" );
  assert( rc2==0 );
 
#define NBYTES 76800
#define CNBYTES 230400
  uint8_t d[NBYTES];
  uint8_t c[CNBYTES];
  uint32_t cntr;


  VideoFrameRef frame, cframe;
  VideoStream** m_streams;
  m_streams = new VideoStream*[2];
  m_streams[0] = &depth;
  m_streams[1] = &color;
  int changedIndex;
  DepthPixel* pDepth;
  uint8_t* pColor;
  int lenPacked;
  while (!wasKeyboardHit())
	{

    rc = OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
    if (rc != STATUS_OK)
    {
      printf("Wait failed\n");
      return 1;
    }

    switch (changedIndex)
    {
      //        fprintf(stdout,"Got depthed\n");
      //        fflush(stdout);
      case 0:
        depth.readFrame(&frame);
        pDepth = (DepthPixel*)frame.getData();
        for(cntr=0;cntr<NBYTES;cntr++)
          d[cntr] = (uint8_t)( pDepth[cntr]/16 );
        zmq_send( socket, (void*)d, NBYTES, 0 );
        break;
      case 1:
        color.readFrame(&cframe);
        pColor = (uint8_t*)cframe.getData();
lenPacked = CompressData( pColor, 320, 240 );
/*
        for(cntr=0;cntr<CNBYTES;cntr+=3)
          c[cntr] = pColor[cntr];
        zmq_send( csocket, (void*)c, CNBYTES, 0 );
*/
zmq_send( csocket, (void*)&(destBuf[0]), lenPacked, 0 );
        break;
      default:
        printf("Error in wait\n");
    }


		//int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
		//printf("[%08llu] %8d %d\n", (long long)frame.getTimestamp(), pDepth[middleIndex], d[middleIndex]);

	}

	color.stop();
	color.destroy();
	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
