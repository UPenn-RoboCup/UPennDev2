/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <libfreenect2/rgb_packet_processor.h>
#include <libfreenect2/async_packet_processor.h>

#include <sys/time.h>
#include <iostream>
//#include <fstream> //#include <string>

namespace libfreenect2
{

RgbPacketProcessor::RgbPacketProcessor() :
    listener_(0)
{
}

RgbPacketProcessor::~RgbPacketProcessor()
{
}

void RgbPacketProcessor::setFrameListener(libfreenect2::FrameListener *listener)
{
  listener_ = listener;
}


class DumpRgbPacketProcessorImpl
{
public:

  Frame *frame;

  double timing_acc;
  double timing_acc_n;

  double timing_current_start;

  DumpRgbPacketProcessorImpl()
  {

    timing_acc = 0.0;
    timing_acc_n = 0.0;
    timing_current_start = 0.0;
  }

  ~DumpRgbPacketProcessorImpl()
  {

  }

  void newFrame(size_t n)
  {
    frame = new Frame(1, 1, n);
  }

  void startTiming()
  {
    static struct timeval t;
    gettimeofday(&t, NULL);
    timing_current_start = t.tv_sec + 1E-6 * t.tv_usec;
  }

  void stopTiming()
  {
    static struct timeval t;
    gettimeofday(&t, NULL);
    double timing_now = t.tv_sec + 1E-6 * t.tv_usec;
    timing_acc += timing_now - timing_current_start;
    timing_acc_n += 1.0;

    if(timing_acc_n >= 100.0)
    {
      double avg = (timing_acc / timing_acc_n);
      std::cout << "[DumpJpegRgbPacketProcessor] avg. time: " << (avg * 1000) << "ms -> ~" << (1.0/avg) << "Hz" << std::endl;
      timing_acc = 0.0;
      timing_acc_n = 0.0;
    }
  }

};


DumpRgbPacketProcessor::DumpRgbPacketProcessor() :
  impl_(new DumpRgbPacketProcessorImpl())
{
}

DumpRgbPacketProcessor::~DumpRgbPacketProcessor()
{
  delete impl_;
}

void DumpRgbPacketProcessor::process(const RgbPacket &packet)
{
  //std::stringstream name;
  //name << packet->sequence << "_" << packet->unknown0 << "_" << jpeg_buffer_length << ".jpeg";
  //
  //std::ofstream file(name.str().c_str());
  //file.write(reinterpret_cast<char *>(packet->jpeg_buffer), jpeg_buffer_length);
  //file.close();

  impl_->newFrame(packet.jpeg_buffer_length);
  std::copy(packet.jpeg_buffer, packet.jpeg_buffer + packet.jpeg_buffer_length, impl_->frame->data);
  
  if(listener_->onNewFrame(Frame::Color, impl_->frame))
  {

  }
  
}

} /* namespace libfreenect2 */
