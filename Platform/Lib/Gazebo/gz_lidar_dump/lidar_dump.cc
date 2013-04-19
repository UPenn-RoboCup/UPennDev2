#include "lidar_dump.h"

#include <zmq.h>
#include <vector>
#include <string>
#include <jpeglib.h>

using namespace gazebo;

void *ctx, *csocket;

std::vector<unsigned char> destBuf;

void LidarDump::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf) {
  // Don't forget to load the camera plugin
  RayPlugin::Load(_parent, _sdf);

  this->parent_sensor_ = _parent;
  this->parent_ray_sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(this->parent_sensor_);

  this->parent_sensor_->SetUpdateRate(40);
  this->parent_ray_sensor_->SetActive(true);
  
  // set up zmq
  ctx = zmq_init(1);
  assert(ctx);
  csocket = zmq_socket(ctx, ZMQ_PUB);
  assert(csocket);
  int rc2 = zmq_bind(csocket, "ipc:///tmp/lidar");
  assert(rc2==0);
}

// Update the controller
void LidarDump::OnNewLaserScans()
{
  struct timeval t;
  gettimeofday(&t, NULL);
//  zmq_send(csocket, (void *)&(destBuf[0]), lenPacked, 0);

  this->parent_ray_sensor_->SetActive(false);
  math::Angle maxAngle = this->parent_ray_sensor_->GetAngleMax();
  math::Angle minAngle = this->parent_ray_sensor_->GetAngleMin();

  double maxRange = this->parent_ray_sensor_->GetRangeMax();
  double minRange = this->parent_ray_sensor_->GetRangeMin();
  int rayCount = this->parent_ray_sensor_->GetRayCount();
  int rangeCount = this->parent_ray_sensor_->GetRangeCount();

  vector<float> ranges;
  ranges.resize(rangeCount);
//  this->parent_ray_sensor_->GetRanges(ranges);
  for (int i = 0; i < rangeCount; i++) {
    ranges[i] = static_cast<float>(this->parent_ray_sensor_->GetRange(i));     
  }

//  std::cout << "Lidar Count: " << rangeCount << ' ' << ranges.size() << endl;

  ostringstream ts_buf;
  ts_buf << std::setw(14) << std::setprecision(15)<< t.tv_sec + 1E-6*t.tv_usec;
  zmq_send(csocket, ts_buf.str().c_str(), ts_buf.str().length(), ZMQ_SNDMORE);
  std::cout << ts_buf.str() << " lidar ts bytes sent: " << ts_buf.str().length() << std::endl;
  zmq_send(csocket, (void *)&(ranges[0]), ranges.size()*sizeof(float), 0); 
  std::cout << " lidar bytes sent!" << std::endl;

  this->parent_ray_sensor_->SetActive(true);
}

