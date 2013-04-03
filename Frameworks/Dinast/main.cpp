#include <string>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>

#include "dinast_grabber.h"

/*
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
*/
 
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

  int d_far = 0;

/*
void
convertImageToCloud (const unsigned char *image, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  cloud.points.resize (IMAGE_WIDTH * IMAGE_HEIGHT);
  cloud.width = IMAGE_WIDTH;
  cloud.height = IMAGE_HEIGHT;
  cloud.is_dense = false;

  int depth_idx = 0;
  int depth_temp[9];
 

  for (int x = 0; x < cloud.width; ++x)
  {
    for (int y = 0; y < cloud.height; ++y, ++depth_idx)
    {
      float xc = (float)(x - 160);
      float yc = (float)(y - 120);
      double r1 = sqrt (xc * xc + yc * yc);
      double r2 = r1 * r1;
      double r3 = r1 * r2;
      double A = -2e-5 * r3 + 0.004 * r2 + 0.1719 * r1 + 350.03;
      double B = -2e-9 * r3 + 3e-7 * r2 - 1e-5 * r1 - 0.01;

      // Low pass filtering
      int measure = 0;
      if ((y > 0) && (y < (IMAGE_HEIGHT - 1)) && (x > 0) && (x < (IMAGE_WIDTH - 1)))
      {
        int ipx = x + IMAGE_WIDTH * y;
        depth_temp[0] = image[ipx];
        depth_temp[1] = image[ipx-1];
        depth_temp[2] = image[ipx+1];
        depth_temp[3] = image[ipx - IMAGE_WIDTH];
        depth_temp[4] = image[ipx - IMAGE_WIDTH - 1];
        depth_temp[5] = image[ipx - IMAGE_WIDTH + 1];
        depth_temp[6] = image[ipx + IMAGE_WIDTH];
        depth_temp[7] = image[ipx + IMAGE_WIDTH - 1];
        depth_temp[8] = image[ipx + IMAGE_WIDTH + 1];

        for (int ii = 0; ii < 9; ii++) 
          measure += depth_temp[ii];
        measure /= 9;
      }

      measure = image[x + IMAGE_WIDTH * y];
      if (measure > 255)  measure = 255;  

      unsigned char depth_256 = measure;

      if (depth_256 < 1) 
      {
        cloud.points[depth_idx].x = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].y = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].z = std::numeric_limits<float>::quiet_NaN ();
        cloud.points[depth_idx].intensity = depth_256;
        continue;
      }

      if (depth_256 > A)   depth_256 = A;


      float dy = y*0.1;
      float dist = (log((double)depth_256/A)/B-dy)*(7E-07*r3 - 0.0001*r2 + 0.004*r1 + 0.9985)*1.5;

      static const double dist_max_2d = 1 / 160.0; 
      static const double FOV = 64.0 * M_PI / 180.0; 
  
      double theta_colati = FOV * r1 * dist_max_2d;
      double c_theta = cos (theta_colati);
      double s_theta = sin (theta_colati);
      double c_ksai = ((double)(x - 160.)) / r1;
      double s_ksai = ((double)(y - 120.)) / r1;

      cloud.points[depth_idx].x = (dist * s_theta * c_ksai);//cartesian x
      cloud.points[depth_idx].y = (dist * s_theta * s_ksai);//cartesian y
      cloud.points[depth_idx].z = (dist * c_theta);                        //cartesian z
      if (cloud.points[depth_idx].z < 1)        cloud.points[depth_idx].z = 1;
      cloud.points[depth_idx].intensity = depth_256;

      if ( depth_256 <15)  //out of effective range
      {
	cloud.points[depth_idx].x /= 10;
	cloud.points[depth_idx].y = 0;
	cloud.points[depth_idx].x += 0.5;
	cloud.points[depth_idx].y += 0.5;
	cloud.points[depth_idx].z = 0;
        cloud.points[depth_idx].intensity = 10;
	d_far++;
      }


    }
  }

}
*/
int
main (int argc, char** argv) 
{
  libusb_context *ctx = NULL;
    struct libusb_device_handle* device = NULL;

  device = findDevice (ctx);
  if (device == NULL)
  {
    std::cerr << "Couldn't find any Dinast devices attached!" << std::endl;
    return (-1);
  } else {
      printf("Found \n");
  }
  /*
  device = openDevice (ctx, 0);
  */
  if (device == NULL)
  {
    std::cerr << "Could not open or claim the USB device!" << std::endl;
    return (-1);
  }

  std::cerr << "Device version/revision number: " << getDeviceVersion (device) << std::endl;
  
  if (!start (device))
    std::cerr << "Could not start device!" << std::endl;

//  if (USBRxSyncSearch (device))
//    std::cerr << "Synchronization succesful." << std::endl;

    /*
  pcl::visualization::ImageViewer vis_img ("Dinast Image Viewer");
  pcl::visualization::PCLVisualizer vis_cld (argc, argv, "Dinast Cloud Viewer");
  pcl::visualization::ImageViewer vis2 ("Dinast Viewer");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
*/
  unsigned char *img1 = (unsigned char*)malloc (IMAGE_SIZE);
  unsigned char *img2 = (unsigned char*)malloc (IMAGE_SIZE);
  printf("Loopy\n");
  while (true)
  {
    if (readImage (device, img1, img2) == 0)
      continue;
    else {
        fprintf(stdout,"Got a reading!!\n");
        fflush(stdout);
    }

	d_far = 0;
  }
  
  stop (device);
  closeDevice (device);
  libusb_exit (ctx);

}

