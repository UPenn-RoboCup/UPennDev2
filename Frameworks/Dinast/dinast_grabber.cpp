#include "dinast_grabber.h"
#include <boost/circular_buffer.hpp>
#include <iostream>

// From http://libusb.sourceforge.net/api-1.0/structlibusb__control__setup.html#a39b148c231d675492ccd2383196926bf
// In: device-to-host.
#define CTRL_TO_HOST   (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
// Out: host-to-device
#define CTRL_TO_DEVICE  (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)

#define CMD_READ_START  0xC7
#define CMD_READ_STOP   0xC8
#define CMD_GET_VERSION 0xDC
#define CMD_SEND_DATA   0xDE

#define SYNC_PACKET     512
#define RGB16_LENGTH    IMAGE_WIDTH*2*IMAGE_HEIGHT

// GLOBAL VARIABLES
/// @todo Should eventually allow opening more than 1 device at once

// Temporary USB read buffer
unsigned char raw_buffer[(RGB16_LENGTH + SYNC_PACKET)*2];

// Global buffer
boost::circular_buffer<unsigned char> g_buffer;

// Bulk endpoint address value
unsigned char bulk_ep = -1;

// Since there is no header after the first image, we need to save the state
bool second_image = false;

  struct libusb_device_handle*
findDevice (libusb_context *ctx, const int id_vendor, const int id_product)
{
  if (libusb_init (&ctx) != 0){
    printf("bad init\n");
    return (NULL);
  }

  //libusb_set_debug (ctx, 3);
  libusb_set_debug (ctx, 0);

  libusb_device **devs = NULL;
  // Get the list of USB devices
  ssize_t cnt = libusb_get_device_list (ctx, &devs);
  if (cnt < 0)
    //throw PCLIOException ();
    return (NULL);

  // Iterate over all devices
  for (ssize_t i = 0; i < cnt; ++i)
  {
    libusb_device_descriptor desc;
    // Get the device descriptor information
    if (libusb_get_device_descriptor (devs[i], &desc) < 0)
    {
      // Free the list
      libusb_free_device_list (devs, 1);
      // Return a NULL device
      return (NULL);
    }

    if (desc.idVendor != id_vendor || desc.idProduct != id_product)
      continue;
    else
      printf("\n\nFound a dinast...\n\n");

    libusb_config_descriptor *config;
    // Get the configuration descriptor
    libusb_get_config_descriptor (devs[i], 0, &config);

    // Iterate over all interfaces available
    for (int i = 0; i < (int)config->bNumInterfaces; ++i)
    {
      // Iterate over the number of alternate settings
      for (int j = 0; j < config->interface[i].num_altsetting; ++j)
      {
        // Iterate over the number of end points present
        for (int k = 0; k < (int)config->interface[i].altsetting[j].bNumEndpoints; ++k) 
        {
          if (config->interface[i].altsetting[j].endpoint[k].bmAttributes == LIBUSB_TRANSFER_TYPE_BULK)
          {
            // Initialize the bulk end point
            bulk_ep = config->interface[i].altsetting[j].endpoint[k].bEndpointAddress;
            break;
          }
        }
      }
    }

    // Free the configuration descriptor
    libusb_free_config_descriptor (config);
  }
  // Free the list
  libusb_free_device_list (devs, 1);

  // No bulk endpoint was found
  if (bulk_ep == -1)
    //throw PCLIOException ();
    return (NULL);

//  struct libusb_device_handle *device;
  printf("opening dev...\n");
  struct libusb_device_handle * device_handle = libusb_open_device_with_vid_pid (ctx, id_vendor, id_product);
  /*
  if (device_handle != NULL)
    libusb_close (device);
  else
    printf("Bad!%x\n",device);
*/
  if (device_handle == NULL)
  {
    printf("NO DIN\n");
    return (NULL);
  } else {
    fprintf(stdout,"Got device handle!\n");
    fflush(stdout);
  }

  // Claim the interface
  int idx = 0;
  if (libusb_claim_interface(device_handle, idx) != 0)
  {
    return (NULL);
  } else {
    fprintf(stdout,"Claimed device handle!\n");
    fflush(stdout);
  }
  
  return (device_handle);
}

  struct libusb_device_handle*
openDevice (struct libusb_context *ctx, int idx)
{

  if (ctx == NULL){
    fprintf(stdout,"Bad initialization of the context!\n");
    fflush(stdout);
    return NULL;
  }
  struct libusb_device_handle * device_handle = libusb_open_device_with_vid_pid (ctx, 0x18d1, 0x1402);

  if (device_handle == NULL)
  {
    //     thrown PCLIOException ();
    return (NULL);
  } else {
    fprintf(stdout,"Got device handle!\n");
    fflush(stdout);
  }

  // Claim the interface
  if (libusb_claim_interface (device_handle, idx) != 0)
  {
    //thrown PCLIOException ();
    return (NULL);
  } else {
    fprintf(stdout,"Claimed device handle!\n");
    fflush(stdout);
  }

  return (device_handle);
}

  void
closeDevice (struct libusb_device_handle* device_handle, int index)
{
  // Release the interface
  libusb_release_interface (device_handle, index);
  // Close it
  libusb_close (device_handle);
}

  bool
USBRxControlData (struct libusb_device_handle* device_handle, 
    const unsigned char req_code,
    unsigned char *buffer,
    int length)
{
  // the direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = CTRL_TO_HOST;
  // the value field for the setup packet
  unsigned int value = 0x02;
  // the index field for the setup packet
  unsigned char index = 0x08;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // for an unlimited timeout, use value 0.
  unsigned int timeout = 1000;

  int nr_read = libusb_control_transfer (device_handle, requesttype, 
      req_code, value, index, buffer, length, timeout);
  if (nr_read != (int)length)
  {
    //throw PCLIOException ();
    return (false);
  }

  return (true);
}

  bool
USBTxControlData (struct libusb_device_handle* device_handle, 
    const unsigned char req_code,
    unsigned char *buffer,
    int length)
{
  // the direction of the transfer is inferred from the requesttype field of the setup packet.
  unsigned char requesttype = CTRL_TO_DEVICE;
  // the value field for the setup packet
  unsigned int value = 0x01;
  // the index field for the setup packet
  unsigned char index = 0x00;
  // timeout (in ms) that this function should wait before giving up due to no response being received
  // for an unlimited timeout, use value 0.
  unsigned int timeout = 1000;

  int nr_read = libusb_control_transfer (device_handle, requesttype, 
      req_code, value, index, buffer, length, timeout);
  if (nr_read != (int)length)
  {
    //throw PCLIOException ();
    return (false);
  }

  return (true);
}

  std::string
getDeviceVersion (struct libusb_device_handle* device_handle)
{
  unsigned char data[30];
  if (!USBRxControlData (device_handle, CMD_GET_VERSION, data, 21))
  {
    //throw PCLIOException ();
    return ("");
  }

  data[21] = 0; // NULL

  return (std::string ((const char*)data));
}

  bool
start (struct libusb_device_handle* device_handle)
{
  unsigned char ctrl_buf[3];
  if (!USBTxControlData (device_handle, CMD_READ_START, ctrl_buf, 1))
    return (false);
  return (true);
}

  bool
stop (struct libusb_device_handle* device_handle)
{
  unsigned char ctrl_buf[3];
  if (!USBTxControlData (device_handle, CMD_READ_STOP, ctrl_buf, 1))
    return (false);
  return (true);
}

  int
checkHeader ()
{
  // We need at least 2 full sync packets, in case the header starts at the end of the first sync packet to 
  // guarantee that the index returned actually exists in g_buffer (we perform no checking in the rest of the code)
  if (g_buffer.size () < 2 * SYNC_PACKET)
    return (-1);

  int data_ptr = -1;

  for (size_t i = 0; i < g_buffer.size (); ++i)
  {
    if ((g_buffer[i+0] == 0xAA) && (g_buffer[i+1] == 0xAA) && 
        (g_buffer[i+2] == 0x44) && (g_buffer[i+3] == 0x44) &&
        (g_buffer[i+4] == 0xBB) && (g_buffer[i+5] == 0xBB) &&
        (g_buffer[i+6] == 0x77) && (g_buffer[i+7] == 0x77))
    {
      data_ptr = i + SYNC_PACKET;
      break;
    }
  }

  return (data_ptr);
}

int readImage (struct libusb_device_handle* device_handle, unsigned char *image) {

  // Do we have enough data in the buffer for the second image?
  if (second_image)
  {
    //std::cerr << "second image copy buf size: " << g_buffer.size () << std::endl;
    // Second image exists. Copy it from the buffer into the user given buffer
    //if (!g_buffer.is_linearized ())
    //  g_buffer.linearize ();
    //memcpy (&image[0], &g_buffer[0], IMAGE_SIZE);
    for (int i = 0; i < IMAGE_SIZE; ++i)
      image[i] = g_buffer[i];

    // Pop the data from the global buffer. 
    //g_buffer.erase_begin (IMAGE_SIZE); // In boost >= 1.42
    g_buffer.rerase (g_buffer.begin(), g_buffer.begin() + IMAGE_SIZE);
    //std::cerr << "second image copy buf size2: " << g_buffer.size () << std::endl;

    second_image = false;
    // Don't do any USB reads, just return
    return (IMAGE_SIZE);
  }

  // Read data in two image blocks until we get a header
  bool first_image = false;
  int data_adr = -1;
  while (!second_image)
  {
    // Read at least two images in synchronous mode
    int actual_length;
    int res = libusb_bulk_transfer (device_handle, bulk_ep, raw_buffer, 
        RGB16_LENGTH + SYNC_PACKET, &actual_length, 1000);
    if (res != 0 || actual_length == 0)
    {
      std::cerr << "USB read error!" << std::endl;
      //throw PCLIOException ();
      memset (&image[0], 0x00, IMAGE_SIZE);
      return (0);
    }

    // Copy data from the USB port if we actually read anything
    //std::cerr << "read " << actual_length << ", buf size: " << g_buffer.size () <<  std::endl;
    // Copy data into the buffer
    int back = g_buffer.size ();
    g_buffer.resize (back + actual_length);
    //memcpy (&g_buffer[back], &raw_buffer[0], actual_length);
    for (int i = 0; i < actual_length; ++i)
      g_buffer[back++] = raw_buffer[i];
    //std::cerr << "buf size: " << g_buffer.size () << std::endl;

    // Check if the header is set already
    if (!first_image && data_adr == -1)
    {
      data_adr = checkHeader ();
      //std::cerr << "search for header: " << data_adr << std::endl;
    }

    // Is there enough data in the buffer to return one image, and did we find the header?
    // If not, go back and read some more data off the USB port
    // Else, read the data, clean the g_buffer, return to the user
    if (!first_image && (g_buffer.size () >= IMAGE_SIZE + data_adr) && data_adr != -1)
    {
      // An image found. Copy it from the buffer into the user given buffer
      //if (!g_buffer.is_linearized ())
      //  g_buffer.linearize ();
      //memcpy (&image[0], &g_buffer[data_adr], IMAGE_SIZE);
      for (int i = 0; i < IMAGE_SIZE; ++i)
        image[i] = g_buffer[data_adr + i];
      // Pop the data from the global buffer. 
      //g_buffer.erase_begin (data_adr + IMAGE_SIZE); // in boost >= 1.42
      g_buffer.rerase (g_buffer.begin(), g_buffer.begin() + data_adr + IMAGE_SIZE);
      first_image = true;
      //std::cerr << "first image copy buf size: " << g_buffer.size () << std::endl;
    }

    if (first_image && g_buffer.size () >= IMAGE_SIZE)
    {
      //second_image = true;
      //std::cerr << "buf size: " << g_buffer.size () << std::endl;
      break;
    }
  }

  return (IMAGE_SIZE);
}

int readImage (struct libusb_device_handle* device_handle,
    unsigned char *image1, unsigned char *image2)
{

  // Read data in two image blocks until we get a header
  int data_adr = -1;
  // Read at least two images in synchronous mode
  int actual_length;
  int res = libusb_bulk_transfer (device_handle, bulk_ep, raw_buffer,
      RGB16_LENGTH * 2 + SYNC_PACKET, &actual_length, 0);
  if (res != 0 || actual_length == 0)
  {
    std::cerr << "USB read error!" << std::endl;
    //throw PCLIOException ();
    memset (&image1[0], 0x00, IMAGE_SIZE);
    return (0);
  }

  for (size_t i = 0; i < actual_length; ++i)
  {
    if ((raw_buffer[i+0] == 0xAA) && (raw_buffer[i+1] == 0xAA) && 
        (raw_buffer[i+2] == 0x44) && (raw_buffer[i+3] == 0x44) &&
        (raw_buffer[i+4] == 0xBB) && (raw_buffer[i+5] == 0xBB) &&
        (raw_buffer[i+6] == 0x77) && (raw_buffer[i+7] == 0x77))
    {
      data_adr = i + SYNC_PACKET;
      break;
    }
  }
  // An image found. Copy it from the buffer into the user given buffer
  for (int i = 0; i < IMAGE_SIZE; ++i)
    image1[i] = raw_buffer[data_adr + i];
  for (int i = 0; i < IMAGE_SIZE; ++i)
    image2[i] = raw_buffer[data_adr + IMAGE_SIZE + i];

  return (IMAGE_SIZE);
}


////////////////////
// Code graveyard //
////////////////////

/// @todo Figure out this sync business
#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Synchronize the data streams and throw away erroneous packets
 * \param[in] device_handle the USB device handle to close
 */
  bool
USBRxSyncSearch (struct libusb_device_handle* device_handle)
{
  unsigned char ctrl_buf[3];
  long raw_length, errlength = 0, header_ptr = 0;

  unsigned char raw_buffer[(RGB16_LENGTH + SYNC_PACKET)*2];

  unsigned int timeout = 0;

  // Remove broken image data in FIFO, errlength
  //  for (int kk = 0; kk < 4; kk++) 
  while (true)
  {
    std::cerr << "Hmm? " << errlength << std::endl;
    raw_length = RGB16_LENGTH + SYNC_PACKET + errlength;// * 2 - (errlength - 8);
    std::cerr << "Reading : " << raw_length << std::endl;
    memset (&raw_buffer[0], 0x00, raw_length);

    int actual_length;

    int res = libusb_bulk_transfer (device_handle, bulk_ep, raw_buffer, 
        raw_length, &actual_length, timeout);
    if (res != 0)
    {
      std::cerr << "RETURNING: " << res << std::endl;
      //throw PCLIOException ();
      return (false);
    }

    // Remove data, if fail to Sync
    //    if (errlength) 
    //      errlength = 0;
    //    else 
    {
      for (long int i = 0; i < actual_length; i++)
      {
        // Check the frame header from the packet 
        if ((raw_buffer[i+0] == 0xAA) && (raw_buffer[i+1] == 0xAA) && 
            (raw_buffer[i+2] == 0x44) && (raw_buffer[i+3] == 0x44) &&
            (raw_buffer[i+4] == 0xBB) && (raw_buffer[i+5] == 0xBB) &&
            (raw_buffer[i+6] == 0x77) && (raw_buffer[i+7] == 0x77))
        {
          // Find the frame header
          // (if normal operation, it is located in the begining >>  header_Ptr = i; = 0)
          header_ptr = i;
          break;
        }
      }

      // If normal operation,  errlength = header_Ptr = 0
      // If something's wrong, errlength = header_Ptr  (need to skip the data next time)
      errlength = header_ptr;  

      std::cerr << "Boundary : " << errlength << std::endl;
      // Align every 512 byte.
      if (header_ptr % SYNC_PACKET) 
      {
        std::cerr << "wtf" << std::endl;
        USBTxControlData (device_handle, CMD_READ_STOP, ctrl_buf, 1);
        usleep (10);
        USBTxControlData (device_handle, CMD_READ_START, ctrl_buf, 1);
      }

      /*      if (header_ptr == 0)
              return (true);
              else 
              return (false);*/
      /*    }
            }

            return (false);
            }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Check if we have a header in the global buffer, and return the position of the next valid image.
 * \note If the image in the buffer is partial, return -1, as we have to wait until we add more data to it.
 * \return the position of the next valid image (i.e., right after a valid header) or -1 in case the buffer 
 * either doesn't have an image or has a partial image
 */
  int
checkHeader ()
{
  if (g_buffer.size () < (IMAGE_SIZE + SYNC_PACKET))
    return (-1);

  int data_ptr = -1;

  for (size_t i = 0; i < g_buffer.size (); ++i)
  {
    if ((g_buffer[i+0] == 0xAA) && (g_buffer[i+1] == 0xAA) && 
        (g_buffer[i+2] == 0x44) && (g_buffer[i+3] == 0x44) &&
        (g_buffer[i+4] == 0xBB) && (g_buffer[i+5] == 0xBB) &&
        (g_buffer[i+6] == 0x77) && (g_buffer[i+7] == 0x77))
    {
      data_ptr = i + SYNC_PACKET;
      break;
    }
  }

  if ((g_buffer.size () - data_ptr) >= IMAGE_SIZE)
    return (data_ptr);
  return (-1);
}
#endif
