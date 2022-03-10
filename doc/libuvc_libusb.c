// code --folder-uri vscode-remote://ssh-remote+synergy/home/dgnet/build/libuvc_make/libuvc
// ssh-copy-id dgnet@192.168.0.103



// open a usb with libusb without uninstalling it's Driver from windows #78
// https://github.com/pbatard/libwdi/issues/78



// https://www.beyondlogic.org/usbnutshell/usb5.shtml

// descriptor
// interface
// endpoint


// All USB devices have a hierarchy of descriptors
// which describe to the host information 
// such as what the device is, who makes it, what version of USB it supports, 
// how many ways it can be configured, 
// the number of endpoints and their types etc.
// USB devices can only have one device descriptor.


// https://www.ontrak.net/c_libusb.htm
// https://github.com/Mathias-L/STM32F4-libusb-example/blob/master/async.c#L254

// Exmaple.
// https://github.com/libusb/libusb/blob/master/examples/sam3u_benchmark.c


// https://www.ontrak.net/c_libusb.htm



//-------------------------------
// home/andriy/Documents/DGC/ArchRepo/DGnet_Dist_PID3/hw/usb/host-libusb.c

// #if LIBUSB_API_VERSION >= 0x01000107 && defined(CONFIG_LINUX) && \
//         defined(USBDEVFS_GET_SPEED)
//         trace_hw_usb_hostlibC_usb_host_open_12_dgtrace();
//     if (hostfd && libusb_speed == 0) {
//         trace_hw_usb_hostlibC_usb_host_open_13_dgtrace();
//         /*
//          * Workaround libusb bug: libusb_get_device_speed() does not
//          * work for libusb_wrap_sys_device() devices in v1.0.23.
//          *
//          * Speeds are defined in linux/usb/ch9.h, file not included
//          * due to name conflicts.
//          */
//-------------------------------





// ====================================================================
// ====================================================================

#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>


int fdnum_yuv;
int fdnum_bgr;

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;

  // sudo ./example 2>/dev/null 1>video.yuv
  // write(1, frame->data, frame->data_bytes);
  write(fdnum_yuv, frame->data, frame->data_bytes);

  enum uvc_frame_format *frame_format = (enum uvc_frame_format *)ptr;
  /* FILE *fp;
   * static int jpeg_count = 0;
   * static const char *H264_FILE = "iOSDevLog.h264";
   * static const char *MJPEG_FILE = ".jpeg";
   * char filename[16]; */

  /* We'll convert the image from YUV/JPEG to BGR, so allocate space */
  bgr = uvc_allocate_frame(frame->width * frame->height * 3);
  if (!bgr) {
    printf("unable to allocate bgr frame!\n");
    return;
  }

  printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %d\n",
    frame->frame_format, frame->width, frame->height, frame->data_bytes, (int) ptr);

  switch (frame->frame_format) {
  case UVC_FRAME_FORMAT_H264:
    /* use `ffplay H264_FILE` to play */
    /* fp = fopen(H264_FILE, "a");
     * fwrite(frame->data, 1, frame->data_bytes, fp);
     * fclose(fp); */
    break;
  case UVC_COLOR_FORMAT_MJPEG:
    /* sprintf(filename, "%d%s", jpeg_count++, MJPEG_FILE);
     * fp = fopen(filename, "w");
     * fwrite(frame->data, 1, frame->data_bytes, fp);
     * fclose(fp); */
    break;
  case UVC_COLOR_FORMAT_YUYV:
    /* Do the BGR conversion */
    ret = uvc_any2bgr(frame, bgr);
    if (ret) {
      uvc_perror(ret, "uvc_any2bgr");
      uvc_free_frame(bgr);
      return;
    }
    write(fdnum_bgr, bgr->data, bgr->data_bytes);
    break;
  default:
    break;
  }

  if (frame->sequence % 30 == 0) {
    printf(" * got image %u\n",  frame->sequence);
  }

  /* Call a user function:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_user_function(ptr, bgr);
   * my_other_function(ptr, bgr->data, bgr->width, bgr->height);
   */

  /* Call a C++ method:
   *
   * my_type *my_obj = (*my_type) ptr;
   * my_obj->my_func(bgr);
   */

  /* Use opencv.highgui to display the image:
   * 
   * cvImg = cvCreateImageHeader(
   *     cvSize(bgr->width, bgr->height),
   *     IPL_DEPTH_8U,
   *     3);
   *
   * cvSetData(cvImg, bgr->data, bgr->width * 3); 
   *
   * cvNamedWindow("Test", CV_WINDOW_AUTOSIZE);
   * cvShowImage("Test", cvImg);
   * cvWaitKey(10);
   *
   * cvReleaseImageHeader(&cvImg);
   */

  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  FILE* h_yuv = fopen("./out.yuv", "a+");
  fdnum_yuv = fileno(h_yuv);

  FILE* h_bgr = fopen("./out.bgr", "a+");
  fdnum_bgr = fileno(h_bgr);


  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx, NULL);
  //start: uvc_init()-----------------------------------------
  //-----------------------------------------
  //-----------------------------------------
      uvc_error_t uvc_init(uvc_context_t **pctx, struct libusb_context *usb_ctx) {
        uvc_error_t ret = UVC_SUCCESS;
        uvc_context_t *ctx = calloc(1, sizeof(*ctx));

        if (usb_ctx == NULL) {
          ret = libusb_init(&ctx->usb_ctx);
          ctx->own_usb_ctx = 1;
          if (ret != UVC_SUCCESS) {
            free(ctx);
            ctx = NULL;
          }
        } else {
          ctx->own_usb_ctx = 0;
          ctx->usb_ctx = usb_ctx;
        }

        if (ctx != NULL)
          *pctx = ctx;

        return ret;
      }
  //end: uvc_init()-----------------------------------------
  //-----------------------------------------
  //-----------------------------------------

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  puts("UVC initialized");

  /* Locates the first attached UVC device, stores in dev */
  res = uvc_find_device(ctx, &dev,
      0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

  //start: uvc_find_device()-----------------------------------------
  //-----------------------------------------
  //-----------------------------------------
      uvc_error_t uvc_find_device(uvc_context_t *ctx, uvc_device_t **dev, int vid, int pid, const char *sn) {

        uvc_error_t ret = UVC_SUCCESS;

        uvc_device_t **list;
        uvc_device_t *test_dev;
        int dev_idx;
        int found_dev;

        UVC_ENTER();

        ret = uvc_get_device_list(ctx, &list);

        //start: uvc_get_device_list()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------
            /**
             * @brief Get a list of the UVC devices attached to the system
             * @ingroup device
             *
             * @note Free the list with uvc_free_device_list when you're done.
             *
             * @param ctx UVC context in which to list devices
             * @param list List of uvc_device structures
             * @return Error if unable to list devices, else SUCCESS
             */
            uvc_error_t uvc_get_device_list(uvc_context_t *ctx, uvc_device_t ***list) {
              struct libusb_device **usb_dev_list;
              struct libusb_device *usb_dev;
              int num_usb_devices;

              uvc_device_t **list_internal;
              int num_uvc_devices;

              /* per device */
              int dev_idx;
              struct libusb_config_descriptor *config;
              struct libusb_device_descriptor desc;
              uint8_t got_interface;

              /* per interface */
              int interface_idx;
              const struct libusb_interface *interface;

              /* per altsetting */
              int altsetting_idx;
              const struct libusb_interface_descriptor *if_desc;

              UVC_ENTER();

              num_usb_devices = libusb_get_device_list(ctx->usb_ctx, &usb_dev_list);

              if (num_usb_devices < 0) {
                UVC_EXIT(UVC_ERROR_IO);
                return UVC_ERROR_IO;
              }

              list_internal = malloc(sizeof(*list_internal));
              *list_internal = NULL;

              num_uvc_devices = 0;
              dev_idx = -1;

              while ((usb_dev = usb_dev_list[++dev_idx]) != NULL) {
                got_interface = 0;

                if (libusb_get_config_descriptor(usb_dev, 0, &config) != 0)
                  continue;

                if ( libusb_get_device_descriptor ( usb_dev, &desc ) != LIBUSB_SUCCESS )
                  continue;

                for (interface_idx = 0; !got_interface && interface_idx < config->bNumInterfaces; ++interface_idx) {
                  interface = &config->interface[interface_idx];

                  for (altsetting_idx = 0; !got_interface && altsetting_idx < interface->num_altsetting;++altsetting_idx) {
                    if_desc = &interface->altsetting[altsetting_idx];

                    // Skip TIS cameras that definitely aren't UVC even though they might
                    // look that way

                    if ( 0x199e == desc.idVendor && desc.idProduct  >= 0x8201 &&
                        desc.idProduct <= 0x8208 ) {
                      continue;
                    }

                    // Special case for Imaging Source cameras
              /* Video, Streaming */
                    if ( 0x199e == desc.idVendor && ( 0x8101 == desc.idProduct ||
                        0x8102 == desc.idProduct ) &&
                        if_desc->bInterfaceClass == 255 &&
                        if_desc->bInterfaceSubClass == 2 ) {
                got_interface = 1;
              }

              /* Video, Streaming */
              if (if_desc->bInterfaceClass == 14 && if_desc->bInterfaceSubClass == 2) {
                got_interface = 1;
              }
                  }
                }

                libusb_free_config_descriptor(config);

                if (got_interface) {
                  uvc_device_t *uvc_dev = malloc(sizeof(*uvc_dev));
                  uvc_dev->ctx = ctx;
                  uvc_dev->ref = 0;
                  uvc_dev->usb_dev = usb_dev;
                  uvc_ref_device(uvc_dev);

                  num_uvc_devices++;
                  list_internal = realloc(list_internal, (num_uvc_devices + 1) * sizeof(*list_internal));

                  list_internal[num_uvc_devices - 1] = uvc_dev;
                  list_internal[num_uvc_devices] = NULL;

                  UVC_DEBUG("    UVC: %d", dev_idx);
                } else {
                  UVC_DEBUG("non-UVC: %d", dev_idx);
                }
              }

              libusb_free_device_list(usb_dev_list, 1);

              *list = list_internal;

              UVC_EXIT(UVC_SUCCESS);
              return UVC_SUCCESS;
            }
        //end: uvc_get_device_list()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------            



        if (ret != UVC_SUCCESS) {
          UVC_EXIT(ret);
          return ret;
        }

        dev_idx = 0;
        found_dev = 0;

        while (!found_dev && (test_dev = list[dev_idx++]) != NULL) {
          uvc_device_descriptor_t *desc;

          if (uvc_get_device_descriptor(test_dev, &desc) != UVC_SUCCESS)
            continue;


          //start: uvc_get_device_descriptor()-----------------------------------------
          //-----------------------------------------
          //-----------------------------------------
              /**
               * @brief Get a descriptor that contains the general information about
               * a device
               * @ingroup device
               *
               * Free *desc with uvc_free_device_descriptor when you're done.
               *
               * @param dev Device to fetch information about
               * @param[out] desc Descriptor structure
               * @return Error if unable to fetch information, else SUCCESS
               */
              uvc_error_t uvc_get_device_descriptor(uvc_device_t *dev, uvc_device_descriptor_t **desc) {

                uvc_device_descriptor_t *desc_internal;
                struct libusb_device_descriptor usb_desc;
                struct libusb_device_handle *usb_devh;
                uvc_error_t ret;

                UVC_ENTER();

                ret = libusb_get_device_descriptor(dev->usb_dev, &usb_desc);

                if (ret != UVC_SUCCESS) {
                  UVC_EXIT(ret);
                  return ret;
                }

                desc_internal = calloc(1, sizeof(*desc_internal));
                desc_internal->idVendor = usb_desc.idVendor;
                desc_internal->idProduct = usb_desc.idProduct;

                if (libusb_open(dev->usb_dev, &usb_devh) == 0) {
                  unsigned char buf[64];

                  int bytes = libusb_get_string_descriptor_ascii(
                      usb_devh, usb_desc.iSerialNumber, buf, sizeof(buf));

                  if (bytes > 0)
                    desc_internal->serialNumber = strdup((const char*) buf);

                  bytes = libusb_get_string_descriptor_ascii(
                      usb_devh, usb_desc.iManufacturer, buf, sizeof(buf));

                  if (bytes > 0)
                    desc_internal->manufacturer = strdup((const char*) buf);

                  bytes = libusb_get_string_descriptor_ascii(
                      usb_devh, usb_desc.iProduct, buf, sizeof(buf));

                  if (bytes > 0)
                    desc_internal->product = strdup((const char*) buf);

                  libusb_close(usb_devh);
                } else {
                  UVC_DEBUG("can't open device %04x:%04x, not fetching serial etc.",
                      usb_desc.idVendor, usb_desc.idProduct);
                }

                *desc = desc_internal;

                UVC_EXIT(ret);
                return ret;
              }

          //end: uvc_get_device_descriptor()-----------------------------------------
          //-----------------------------------------
          //-----------------------------------------  


          if ((!vid || desc->idVendor == vid)
              && (!pid || desc->idProduct == pid)
              && (!sn || (desc->serialNumber && !strcmp(desc->serialNumber, sn))))
            found_dev = 1;

          uvc_free_device_descriptor(desc);
        }

        if (found_dev)
          uvc_ref_device(test_dev);

        uvc_free_device_list(list, 1);

        if (found_dev) {
          *dev = test_dev;
          UVC_EXIT(UVC_SUCCESS);
          return UVC_SUCCESS;
        } else {
          UVC_EXIT(UVC_ERROR_NO_DEVICE);
          return UVC_ERROR_NO_DEVICE;
        }

      }
  //end: uvc_find_device()-----------------------------------------
  //-----------------------------------------
  //-----------------------------------------

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    puts("Device found");

    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);

    //start: uvc_open()-----------------------------------------
    //-----------------------------------------
    //-----------------------------------------

      /** @brief Open a UVC device
       * @ingroup device
       *
       * @param dev Device to open
       * @param[out] devh Handle on opened device
       * @return Error opening device or SUCCESS
       */
      uvc_error_t uvc_open(uvc_device_t *dev, uvc_device_handle_t **devh) {
        
        uvc_error_t ret;
        struct libusb_device_handle *usb_devh;

        UVC_ENTER();

        ret = libusb_open(dev->usb_dev, &usb_devh);
        UVC_DEBUG("libusb_open() = %d", ret);

        if (ret != UVC_SUCCESS) {
          UVC_EXIT(ret);
          return ret;
        }

        ret = uvc_open_internal(dev, usb_devh, devh);

        //start: uvc_open_internal()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------

          static uvc_error_t uvc_open_internal(
              uvc_device_t *dev,
              struct libusb_device_handle *usb_devh,
              uvc_device_handle_t **devh) {
            uvc_error_t ret;
            uvc_device_handle_t *internal_devh;
            struct libusb_device_descriptor desc;

            UVC_ENTER();

            uvc_ref_device(dev);

            internal_devh = calloc(1, sizeof(*internal_devh));
            internal_devh->dev = dev;
            internal_devh->usb_devh = usb_devh;

            ret = uvc_get_device_info(internal_devh, &(internal_devh->info));

            if (ret != UVC_SUCCESS)
              goto fail;

            UVC_DEBUG("claiming control interface %d", internal_devh->info->ctrl_if.bInterfaceNumber);
            ret = uvc_claim_if(internal_devh, internal_devh->info->ctrl_if.bInterfaceNumber);
            if (ret != UVC_SUCCESS)
              goto fail;

            libusb_get_device_descriptor(dev->usb_dev, &desc);
            internal_devh->is_isight = (desc.idVendor == 0x05ac && desc.idProduct == 0x8501);

            if (internal_devh->info->ctrl_if.bEndpointAddress) {
              internal_devh->status_xfer = libusb_alloc_transfer(0);
              if (!internal_devh->status_xfer) {
                ret = UVC_ERROR_NO_MEM;
                goto fail;
              }

              libusb_fill_interrupt_transfer(internal_devh->status_xfer,
                                            usb_devh,
                                            internal_devh->info->ctrl_if.bEndpointAddress,
                                            internal_devh->status_buf,
                                            sizeof(internal_devh->status_buf),
                                            _uvc_status_callback,
                                            internal_devh,
                                            0);
              ret = libusb_submit_transfer(internal_devh->status_xfer);
              UVC_DEBUG("libusb_submit_transfer() = %d", ret);

              if (ret) {
                fprintf(stderr,
                        "uvc: device has a status interrupt endpoint, but unable to read from it\n");
                goto fail;
              }
            }

            if (dev->ctx->own_usb_ctx && dev->ctx->open_devices == NULL) {
              /* Since this is our first device, we need to spawn the event handler thread */
              uvc_start_handler_thread(dev->ctx);

            //start: uvc_start_handler_thread()-----------------------------------------
            //-----------------------------------------
            //-----------------------------------------
              /**
               * @internal
               * @brief Spawns a handler thread for the context
               * @ingroup init
               *
               * This should be called at the end of a successful uvc_open if no devices
               * are already open (and being handled).
               */
              void uvc_start_handler_thread(uvc_context_t *ctx) {
                if (ctx->own_usb_ctx)
                  pthread_create(&ctx->handler_thread, NULL, _uvc_handle_events, (void*) ctx);

                  //start: _uvc_handle_events-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------
                    /** @internal
                     * @brief Event handler thread
                     * There's one of these per UVC context.
                     * @todo We shouldn't run this if we don't own the USB context
                     */
                    void *_uvc_handle_events(void *arg) {
                      uvc_context_t *ctx = (uvc_context_t *) arg;

                      while (!ctx->kill_handler_thread)
                        libusb_handle_events_completed(ctx->usb_ctx, &ctx->kill_handler_thread);
                      return NULL;
                    }
                  //end: _uvc_handle_events-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------
              }
            //end: uvc_start_handler_thread()-----------------------------------------
            //-----------------------------------------
            //-----------------------------------------


            }

            DL_APPEND(dev->ctx->open_devices, internal_devh);
            *devh = internal_devh;

            UVC_EXIT(ret);

            return ret;

          fail:
            if ( internal_devh->info ) {
              uvc_release_if(internal_devh, internal_devh->info->ctrl_if.bInterfaceNumber);
            }
            libusb_close(usb_devh);
            uvc_unref_device(dev);
            uvc_free_devh(internal_devh);

            UVC_EXIT(ret);

            return ret;
          }

        //end: uvc_open_internal()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------


        UVC_EXIT(ret);
        return ret;
      }


    //end: uvc_open()-----------------------------------------
    //-----------------------------------------
    //-----------------------------------------



    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      puts("Device opened");

      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      //start: uvc_print_diag()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------

          /** @brief Print camera capabilities and configuration.
           * @ingroup diag
           *
           * @param devh UVC device
           * @param stream Output stream (stderr if NULL)
           */
          void uvc_print_diag(uvc_device_handle_t *devh, FILE *stream) {
            if (stream == NULL)
              stream = stderr;

            if (devh->info->ctrl_if.bcdUVC) {
              uvc_streaming_interface_t *stream_if;
              int stream_idx = 0;

              uvc_device_descriptor_t *desc;
              uvc_get_device_descriptor(devh->dev, &desc);

              fprintf(stream, "DEVICE CONFIGURATION (%04x:%04x/%s) ---\n",
                  desc->idVendor, desc->idProduct,
                  desc->serialNumber ? desc->serialNumber : "[none]");

              uvc_free_device_descriptor(desc);

              fprintf(stream, "Status: %s\n", devh->streams ? "streaming" : "idle");

              fprintf(stream, "VideoControl:\n"
                  "\tbcdUVC: 0x%04x\n",
                  devh->info->ctrl_if.bcdUVC);

              DL_FOREACH(devh->info->stream_ifs, stream_if) {
                uvc_format_desc_t *fmt_desc;

                ++stream_idx;

                fprintf(stream, "VideoStreaming(%d):\n"
                    "\tbEndpointAddress: %d\n\tFormats:\n",
                    stream_idx, stream_if->bEndpointAddress);

                DL_FOREACH(stream_if->format_descs, fmt_desc) {
                  uvc_frame_desc_t *frame_desc;
                  int i;

                  switch (fmt_desc->bDescriptorSubtype) {
                    case UVC_VS_FORMAT_UNCOMPRESSED:
                    case UVC_VS_FORMAT_MJPEG:
                    case UVC_VS_FORMAT_FRAME_BASED:
                      fprintf(stream,
                          "\t\%s(%d)\n"
                          "\t\t  bits per pixel: %d\n"
                          "\t\t  GUID: ",
                          _uvc_name_for_format_subtype(fmt_desc->bDescriptorSubtype),
                          fmt_desc->bFormatIndex,
                          fmt_desc->bBitsPerPixel);

                      for (i = 0; i < 16; ++i)
                        fprintf(stream, "%02x", fmt_desc->guidFormat[i]);

                      fprintf(stream, " (%4s)\n", fmt_desc->fourccFormat );

                      fprintf(stream,
                          "\t\t  default frame: %d\n"
                          "\t\t  aspect ratio: %dx%d\n"
                          "\t\t  interlace flags: %02x\n"
                          "\t\t  copy protect: %02x\n",
                          fmt_desc->bDefaultFrameIndex,
                          fmt_desc->bAspectRatioX,
                          fmt_desc->bAspectRatioY,
                          fmt_desc->bmInterlaceFlags,
                          fmt_desc->bCopyProtect);

                      DL_FOREACH(fmt_desc->frame_descs, frame_desc) {
                        uint32_t *interval_ptr;

                        fprintf(stream,
                            "\t\t\tFrameDescriptor(%d)\n"
                            "\t\t\t  capabilities: %02x\n"
                            "\t\t\t  size: %dx%d\n"
                            "\t\t\t  bit rate: %d-%d\n"
                            "\t\t\t  max frame size: %d\n"
                            "\t\t\t  default interval: 1/%d\n",
                            frame_desc->bFrameIndex,
                            frame_desc->bmCapabilities,
                            frame_desc->wWidth,
                            frame_desc->wHeight,
                            frame_desc->dwMinBitRate,
                            frame_desc->dwMaxBitRate,
                            frame_desc->dwMaxVideoFrameBufferSize,
                            10000000 / frame_desc->dwDefaultFrameInterval);
                        if (frame_desc->intervals) {
                          for (interval_ptr = frame_desc->intervals;
                              *interval_ptr;
                              ++interval_ptr) {
                            fprintf(stream,
                                "\t\t\t  interval[%d]: 1/%d\n",
                    (int) (interval_ptr - frame_desc->intervals),
                    10000000 / *interval_ptr);
                          }
                        } else {
                          fprintf(stream,
                              "\t\t\t  min interval[%d] = 1/%d\n"
                              "\t\t\t  max interval[%d] = 1/%d\n",
                              frame_desc->dwMinFrameInterval,
                              10000000 / frame_desc->dwMinFrameInterval,
                              frame_desc->dwMaxFrameInterval,
                              10000000 / frame_desc->dwMaxFrameInterval);
                          if (frame_desc->dwFrameIntervalStep)
                            fprintf(stream,
                                "\t\t\t  interval step[%d] = 1/%d\n",
                                frame_desc->dwFrameIntervalStep,
                                10000000 / frame_desc->dwFrameIntervalStep);
                        }
                      }
                      if(fmt_desc->still_frame_desc)
                      {
                          uvc_still_frame_desc_t* still_frame_desc;
                          DL_FOREACH(fmt_desc->still_frame_desc, still_frame_desc)
                          {
                              fprintf(stream,
                                  "\t\t\tStillFrameDescriptor\n"
                                  "\t\t\t  bEndPointAddress: %02x\n",
                                  still_frame_desc->bEndPointAddress);
                              uvc_still_frame_res_t* imageSizePattern;
                              DL_FOREACH(still_frame_desc->imageSizePatterns, imageSizePattern) {
                                  fprintf(stream,
                                      "\t\t\t  wWidth(%d) = %d\n"
                                      "\t\t\t  wHeight(%d) = %d\n",
                                      imageSizePattern->bResolutionIndex,
                                      imageSizePattern->wWidth,
                                      imageSizePattern->bResolutionIndex,
                                      imageSizePattern->wHeight);
                              }
                          }
                      }
                      break;
                    default:
                      fprintf(stream, "\t-UnknownFormat (%d)\n",
                          fmt_desc->bDescriptorSubtype );
                  }
                }
              }

              fprintf(stream, "END DEVICE CONFIGURATION\n");
            } else {
              fprintf(stream, "uvc_print_diag: Device not configured!\n");
            }
          }

      //end: uvc_print_diag()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);

      //start: uvc_get_format_descs()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------

        /**
         * @brief Get format descriptions for the open device.
         *
         * @note Do not modify the returned structure.
         *
         * @param devh Device handle to an open UVC device
         */
        const uvc_format_desc_t *uvc_get_format_descs(uvc_device_handle_t *devh) {
          return devh->info->stream_ifs->format_descs;
        }

      //end: uvc_get_format_descs()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------


      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
      //frame_desc = frame_desc->next; // 640x480

      enum uvc_frame_format frame_format;
      int width = 640;
      int height = 480;
      int fps = 30;

      switch (format_desc->bDescriptorSubtype) {
      case UVC_VS_FORMAT_MJPEG:
        frame_format = UVC_COLOR_FORMAT_MJPEG;
        break;
      case UVC_VS_FORMAT_FRAME_BASED:
        frame_format = UVC_FRAME_FORMAT_H264;
        break;
      default:
        frame_format = UVC_FRAME_FORMAT_YUYV;
        break;
      }

      if (frame_desc) {
        width = frame_desc->wWidth;
        height = frame_desc->wHeight;
        fps = 10000000 / frame_desc->dwDefaultFrameInterval;
      }

      printf("\nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat, width, height, fps);

      /* Try to negotiate first stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, /* result stored in ctrl */
          frame_format,
          width, height, fps /* width, height, fps */
      );

      //start: uvc_get_stream_ctrl_format_size()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------

          /** Get a negotiated streaming control block for some common parameters.
           * @ingroup streaming
           *
           * @param[in] devh Device handle
           * @param[in,out] ctrl Control block
           * @param[in] format_class Type of streaming format
           * @param[in] width Desired frame width
           * @param[in] height Desired frame height
           * @param[in] fps Frame rate, frames per second
           */
          uvc_error_t uvc_get_stream_ctrl_format_size(
              uvc_device_handle_t *devh,
              uvc_stream_ctrl_t *ctrl,
              enum uvc_frame_format cf,
              int width, int height,
              int fps) {
            uvc_streaming_interface_t *stream_if;

            /* find a matching frame descriptor and interval */
            DL_FOREACH(devh->info->stream_ifs, stream_if) {
              uvc_format_desc_t *format;

              DL_FOREACH(stream_if->format_descs, format) {
                uvc_frame_desc_t *frame;

                if (!_uvc_frame_format_matches_guid(cf, format->guidFormat))
                  continue;

                DL_FOREACH(format->frame_descs, frame) {
                  if (frame->wWidth != width || frame->wHeight != height)
                    continue;

                  uint32_t *interval;

                  ctrl->bInterfaceNumber = stream_if->bInterfaceNumber;
                  UVC_DEBUG("claiming streaming interface %d", stream_if->bInterfaceNumber );
                  uvc_claim_if(devh, ctrl->bInterfaceNumber);
                  /* get the max values */
                  uvc_query_stream_ctrl( devh, ctrl, 1, UVC_GET_MAX);

                  if (frame->intervals) {
                    for (interval = frame->intervals; *interval; ++interval) {
                      // allow a fps rate of zero to mean "accept first rate available"
                      if (10000000 / *interval == (unsigned int) fps || fps == 0) {

                        ctrl->bmHint = (1 << 0); /* don't negotiate interval */
                        ctrl->bFormatIndex = format->bFormatIndex;
                        ctrl->bFrameIndex = frame->bFrameIndex;
                        ctrl->dwFrameInterval = *interval;

                        goto found;
                      }
                    }
                  } else {
                    uint32_t interval_100ns = 10000000 / fps;
                    uint32_t interval_offset = interval_100ns - frame->dwMinFrameInterval;

                    if (interval_100ns >= frame->dwMinFrameInterval
                        && interval_100ns <= frame->dwMaxFrameInterval
                        && !(interval_offset
                            && (interval_offset % frame->dwFrameIntervalStep))) {

                      ctrl->bmHint = (1 << 0);
                      ctrl->bFormatIndex = format->bFormatIndex;
                      ctrl->bFrameIndex = frame->bFrameIndex;
                      ctrl->dwFrameInterval = interval_100ns;

                      goto found;
                    }
                  }
                }
              }
            }

            return UVC_ERROR_INVALID_MODE;

          found:
            return uvc_probe_stream_ctrl(devh, ctrl);
          }
      //end: uvc_get_stream_ctrl_format_size()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------



      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);


      //start: uvc_print_stream_ctrl()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------

          /** @brief Print the values in a stream control block
           * @ingroup diag
           *
           * @param devh UVC device
           * @param stream Output stream (stderr if NULL)
           */
          void uvc_print_stream_ctrl(uvc_stream_ctrl_t *ctrl, FILE *stream) {
            if (stream == NULL)
              stream = stderr;

            fprintf(stream, "bmHint: %04x\n", ctrl->bmHint);
            fprintf(stream, "bFormatIndex: %d\n", ctrl->bFormatIndex);
            fprintf(stream, "bFrameIndex: %d\n", ctrl->bFrameIndex);
            fprintf(stream, "dwFrameInterval: %u\n", ctrl->dwFrameInterval);
            fprintf(stream, "wKeyFrameRate: %d\n", ctrl->wKeyFrameRate);
            fprintf(stream, "wPFrameRate: %d\n", ctrl->wPFrameRate);
            fprintf(stream, "wCompQuality: %d\n", ctrl->wCompQuality);
            fprintf(stream, "wCompWindowSize: %d\n", ctrl->wCompWindowSize);
            fprintf(stream, "wDelay: %d\n", ctrl->wDelay);
            fprintf(stream, "dwMaxVideoFrameSize: %u\n", ctrl->dwMaxVideoFrameSize);
            fprintf(stream, "dwMaxPayloadTransferSize: %u\n", ctrl->dwMaxPayloadTransferSize);
            fprintf(stream, "bInterfaceNumber: %d\n", ctrl->bInterfaceNumber);
          }


      //end: uvc_print_stream_ctrl()-----------------------------------------
      //-----------------------------------------
      //-----------------------------------------



      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void *) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void *) 12345, 0);

        //start: uvc_start_streaming()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------

            /** Begin streaming video from the camera into the callback function.
             * @ingroup streaming
             *
             * @param devh UVC device
             * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
             *             {uvc_get_stream_ctrl_format_size}
             * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
             * @param flags Stream setup flags, currently undefined. Set this to zero. The lower bit
             * is reserved for backward compatibility.
             */
            uvc_error_t uvc_start_streaming(
                uvc_device_handle_t *devh,
                uvc_stream_ctrl_t *ctrl,
                uvc_frame_callback_t *cb,
                void *user_ptr,
                uint8_t flags
            ) {
              uvc_error_t ret;
              uvc_stream_handle_t *strmh;

              ret = uvc_stream_open_ctrl(devh, &strmh, ctrl);

              //start: uvc_stream_open_ctrl()-----------------------------------------
              //-----------------------------------------
              //-----------------------------------------

                  /** Open a new video stream.
                   * @ingroup streaming
                   *
                   * @param devh UVC device
                   * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
                   *             {uvc_get_stream_ctrl_format_size}
                   */
                  uvc_error_t uvc_stream_open_ctrl(uvc_device_handle_t *devh, uvc_stream_handle_t **strmhp, uvc_stream_ctrl_t *ctrl) {
                    /* Chosen frame and format descriptors */
                    uvc_stream_handle_t *strmh = NULL;
                    uvc_streaming_interface_t *stream_if;
                    uvc_error_t ret;

                    UVC_ENTER();

                    if (_uvc_get_stream_by_interface(devh, ctrl->bInterfaceNumber) != NULL) {
                      ret = UVC_ERROR_BUSY; /* Stream is already opened */
                      goto fail;
                    }

                    //start: _uvc_get_stream_by_interface()-----------------------------------------
                    //-----------------------------------------
                    //-----------------------------------------


                      static uvc_stream_handle_t *_uvc_get_stream_by_interface(uvc_device_handle_t *devh, int interface_idx) {
                        uvc_stream_handle_t *strmh;

                        DL_FOREACH(devh->streams, strmh) {
                          if (strmh->stream_if->bInterfaceNumber == interface_idx)
                            return strmh;
                        }

                        return NULL;
                      }


                  //end: _uvc_get_stream_by_interface()-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------




                    stream_if = _uvc_get_stream_if(devh, ctrl->bInterfaceNumber);
                    if (!stream_if) {
                      ret = UVC_ERROR_INVALID_PARAM;
                      goto fail;
                    }


                  //start:_uvc_get_stream_if()-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------

                    static uvc_streaming_interface_t *_uvc_get_stream_if(uvc_device_handle_t *devh, int interface_idx) {
                      uvc_streaming_interface_t *stream_if;

                      DL_FOREACH(devh->info->stream_ifs, stream_if) {
                        if (stream_if->bInterfaceNumber == interface_idx)
                          return stream_if;
                      }
                      
                      return NULL;
                    }

                  //end: _uvc_get_stream_if()-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------


                    strmh = calloc(1, sizeof(*strmh));
                    if (!strmh) {
                      ret = UVC_ERROR_NO_MEM;
                      goto fail;
                    }
                    strmh->devh = devh;
                    strmh->stream_if = stream_if;
                    strmh->frame.library_owns_data = 1;

                    ret = uvc_claim_if(strmh->devh, strmh->stream_if->bInterfaceNumber);
                    if (ret != UVC_SUCCESS)
                      goto fail;


                    //start: uvc_claim_if()-----------------------------------------
                    //-----------------------------------------
                    //-----------------------------------------
                      /** @internal
                       * Claim a UVC interface, detaching the kernel driver if necessary.
                       * @ingroup device
                       *
                       * @param devh UVC device handle
                       * @param idx UVC interface index
                       */
                      uvc_error_t uvc_claim_if(uvc_device_handle_t *devh, int idx) {
                        int ret = UVC_SUCCESS;

                        UVC_ENTER();

                        if ( devh->claimed & ( 1 << idx )) {
                          UVC_DEBUG("attempt to claim already-claimed interface %d\n", idx );
                          UVC_EXIT(ret);
                          return ret;
                        }

                        /* Tell libusb to detach any active kernel drivers. libusb will keep track of whether
                        * it found a kernel driver for this interface. */
                        ret = libusb_detach_kernel_driver(devh->usb_devh, idx);

                        if (ret == UVC_SUCCESS || ret == LIBUSB_ERROR_NOT_FOUND || ret == LIBUSB_ERROR_NOT_SUPPORTED) {
                          UVC_DEBUG("claiming interface %d", idx);
                          if (!( ret = libusb_claim_interface(devh->usb_devh, idx))) {
                            devh->claimed |= ( 1 << idx );
                          }
                        } else {
                          UVC_DEBUG("not claiming interface %d: unable to detach kernel driver (%s)",
                                    idx, uvc_strerror(ret));
                        }

                        UVC_EXIT(ret);
                        return ret;
                      }


                    //end: uvc_claim_if()-----------------------------------------
                    //-----------------------------------------
                    //-----------------------------------------



                    ret = uvc_stream_ctrl(strmh, ctrl);
                    if (ret != UVC_SUCCESS)
                      goto fail;

                    //start: uvc_stream_ctrl()-----------------------------------------
                    //-----------------------------------------
                    //-----------------------------------------
                      /** @brief Reconfigure stream with a new stream format.
                       * @ingroup streaming
                       *
                       * This may be executed whether or not the stream is running.
                       *
                       * @param[in] strmh Stream handle
                       * @param[in] ctrl Control block, processed using {uvc_probe_stream_ctrl} or
                       *             {uvc_get_stream_ctrl_format_size}
                       */
                      uvc_error_t uvc_stream_ctrl(uvc_stream_handle_t *strmh, uvc_stream_ctrl_t *ctrl) {
                        uvc_error_t ret;

                        if (strmh->stream_if->bInterfaceNumber != ctrl->bInterfaceNumber)
                          return UVC_ERROR_INVALID_PARAM;

                        /* @todo Allow the stream to be modified without restarting the stream */
                        if (strmh->running)
                          return UVC_ERROR_BUSY;

                        ret = uvc_query_stream_ctrl(strmh->devh, ctrl, 0, UVC_SET_CUR);

                        //start: uvc_query_stream_ctrl()-----------------------------------------
                        //-----------------------------------------
                        //-----------------------------------------
                          /** @internal
                           * Run a streaming control query
                           * @param[in] devh UVC device
                           * @param[in,out] ctrl Control block
                           * @param[in] probe Whether this is a probe query or a commit query
                           * @param[in] req Query type
                           */
                          uvc_error_t uvc_query_stream_ctrl(
                              uvc_device_handle_t *devh,
                              uvc_stream_ctrl_t *ctrl,
                              uint8_t probe,
                              enum uvc_req_code req) {
                            uint8_t buf[34];
                            size_t len;
                            uvc_error_t err;

                            memset(buf, 0, sizeof(buf));

                            if (devh->info->ctrl_if.bcdUVC >= 0x0110)
                              len = 34;
                            else
                              len = 26;

                            /* prepare for a SET transfer */
                            if (req == UVC_SET_CUR) {
                              SHORT_TO_SW(ctrl->bmHint, buf);
                              buf[2] = ctrl->bFormatIndex;
                              buf[3] = ctrl->bFrameIndex;
                              INT_TO_DW(ctrl->dwFrameInterval, buf + 4);
                              SHORT_TO_SW(ctrl->wKeyFrameRate, buf + 8);
                              SHORT_TO_SW(ctrl->wPFrameRate, buf + 10);
                              SHORT_TO_SW(ctrl->wCompQuality, buf + 12);
                              SHORT_TO_SW(ctrl->wCompWindowSize, buf + 14);
                              SHORT_TO_SW(ctrl->wDelay, buf + 16);
                              INT_TO_DW(ctrl->dwMaxVideoFrameSize, buf + 18);
                              INT_TO_DW(ctrl->dwMaxPayloadTransferSize, buf + 22);

                              if (len == 34) {
                                INT_TO_DW ( ctrl->dwClockFrequency, buf + 26 );
                                buf[30] = ctrl->bmFramingInfo;
                                buf[31] = ctrl->bPreferredVersion;
                                buf[32] = ctrl->bMinVersion;
                                buf[33] = ctrl->bMaxVersion;
                                /** @todo support UVC 1.1 */
                              }
                            }

                            /* do the transfer */
                            err = libusb_control_transfer(
                                devh->usb_devh,
                                req == UVC_SET_CUR ? 0x21 : 0xA1,
                                req,
                                probe ? (UVC_VS_PROBE_CONTROL << 8) : (UVC_VS_COMMIT_CONTROL << 8),
                                ctrl->bInterfaceNumber,
                                buf, len, 0
                            );

                            if (err <= 0) {
                              return err;
                            }

                            /* now decode following a GET transfer */
                            if (req != UVC_SET_CUR) {
                              ctrl->bmHint = SW_TO_SHORT(buf);
                              ctrl->bFormatIndex = buf[2];
                              ctrl->bFrameIndex = buf[3];
                              ctrl->dwFrameInterval = DW_TO_INT(buf + 4);
                              ctrl->wKeyFrameRate = SW_TO_SHORT(buf + 8);
                              ctrl->wPFrameRate = SW_TO_SHORT(buf + 10);
                              ctrl->wCompQuality = SW_TO_SHORT(buf + 12);
                              ctrl->wCompWindowSize = SW_TO_SHORT(buf + 14);
                              ctrl->wDelay = SW_TO_SHORT(buf + 16);
                              ctrl->dwMaxVideoFrameSize = DW_TO_INT(buf + 18);
                              ctrl->dwMaxPayloadTransferSize = DW_TO_INT(buf + 22);

                              if (len == 34) {
                                ctrl->dwClockFrequency = DW_TO_INT ( buf + 26 );
                                ctrl->bmFramingInfo = buf[30];
                                ctrl->bPreferredVersion = buf[31];
                                ctrl->bMinVersion = buf[32];
                                ctrl->bMaxVersion = buf[33];
                                /** @todo support UVC 1.1 */
                              }
                              else
                                ctrl->dwClockFrequency = devh->info->ctrl_if.dwClockFrequency;

                              /* fix up block for cameras that fail to set dwMax* */
                              if (ctrl->dwMaxVideoFrameSize == 0) {
                                uvc_frame_desc_t *frame = uvc_find_frame_desc(devh, ctrl->bFormatIndex, ctrl->bFrameIndex);

                                //start: uvc_find_frame_desc()-----------------------------------------
                                //-----------------------------------------
                                //-----------------------------------------
                                    /** @internal
                                     * @brief Find the descriptor for a specific frame configuration
                                     * @param devh UVC device
                                     * @param format_id Index of format class descriptor
                                     * @param frame_id Index of frame descriptor
                                     */
                                    uvc_frame_desc_t *uvc_find_frame_desc(uvc_device_handle_t *devh,
                                        uint16_t format_id, uint16_t frame_id) {
                                    
                                      uvc_streaming_interface_t *stream_if;
                                      uvc_frame_desc_t *frame;

                                      DL_FOREACH(devh->info->stream_ifs, stream_if) {
                                        frame = _uvc_find_frame_desc_stream_if(stream_if, format_id, frame_id);

                                        //start: _uvc_find_frame_desc_stream_if()-----------------------------------------
                                        //-----------------------------------------
                                        //-----------------------------------------
                                          /** @internal
                                           * @brief Find the descriptor for a specific frame configuration
                                           * @param stream_if Stream interface
                                           * @param format_id Index of format class descriptor
                                           * @param frame_id Index of frame descriptor
                                           */
                                          static uvc_frame_desc_t *_uvc_find_frame_desc_stream_if(uvc_streaming_interface_t *stream_if,
                                              uint16_t format_id, uint16_t frame_id) {
                                          
                                            uvc_format_desc_t *format = NULL;
                                            uvc_frame_desc_t *frame = NULL;

                                            DL_FOREACH(stream_if->format_descs, format) {
                                              if (format->bFormatIndex == format_id) {
                                                DL_FOREACH(format->frame_descs, frame) {
                                                  if (frame->bFrameIndex == frame_id)
                                                    return frame;
                                                }
                                              }
                                            }

                                            return NULL;
                                          }
                                        //stop: _uvc_find_frame_desc_stream_if()-----------------------------------------
                                        //-----------------------------------------
                                        //-----------------------------------------

                                        if (frame)
                                          return frame;
                                      }

                                      return NULL;
                                    }
                                //end: uvc_find_frame_desc()-----------------------------------------
                                //-----------------------------------------
                                //-----------------------------------------




                                if (frame) {
                                  ctrl->dwMaxVideoFrameSize = frame->dwMaxVideoFrameBufferSize;
                                }
                              }
                            }

                            return UVC_SUCCESS;
                          }
                        //end: uvc_query_stream_ctrl()-----------------------------------------
                        //-----------------------------------------
                        //-----------------------------------------


                        if (ret != UVC_SUCCESS)
                          return ret;

                        strmh->cur_ctrl = *ctrl;
                        return UVC_SUCCESS;
                      }
                    //end: uvc_stream_ctrl()-----------------------------------------
                    //-----------------------------------------
                    //-----------------------------------------

                    // Set up the streaming status and data space
                    strmh->running = 0;

                    strmh->outbuf = malloc( ctrl->dwMaxVideoFrameSize );
                    strmh->holdbuf = malloc( ctrl->dwMaxVideoFrameSize );

                    strmh->meta_outbuf = malloc( LIBUVC_XFER_META_BUF_SIZE );
                    strmh->meta_holdbuf = malloc( LIBUVC_XFER_META_BUF_SIZE );
                    
                    pthread_mutex_init(&strmh->cb_mutex, NULL);
                    pthread_cond_init(&strmh->cb_cond, NULL);

                    DL_APPEND(devh->streams, strmh);

                    *strmhp = strmh;

                    UVC_EXIT(0);
                    return UVC_SUCCESS;

                  fail:
                    if(strmh)
                      free(strmh);
                    UVC_EXIT(ret);
                    return ret;
                  }
              //end: uvc_stream_open_ctrl()-----------------------------------------
              //-----------------------------------------
              //-----------------------------------------

              if (ret != UVC_SUCCESS)
                return ret;

              ret = uvc_stream_start(strmh, cb, user_ptr, flags);

              //start: uvc_stream_start()-----------------------------------------
              //-----------------------------------------
              //-----------------------------------------
                /** Begin streaming video from the stream into the callback function.
                 * @ingroup streaming
                 *
                 * @param strmh UVC stream
                 * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
                 * @param flags Stream setup flags, currently undefined. Set this to zero. The lower bit
                 * is reserved for backward compatibility.
                 */
                uvc_error_t uvc_stream_start(
                    uvc_stream_handle_t *strmh,
                    uvc_frame_callback_t *cb,
                    void *user_ptr,
                    uint8_t flags
                ) {
                  /* USB interface we'll be using */
                  const struct libusb_interface *interface;
                  int interface_id;
                  char isochronous;
                  uvc_frame_desc_t *frame_desc;
                  uvc_format_desc_t *format_desc;
                  uvc_stream_ctrl_t *ctrl;
                  uvc_error_t ret;
                  /* Total amount of data per transfer */
                  size_t total_transfer_size = 0;
                  struct libusb_transfer *transfer;
                  int transfer_id;

                  ctrl = &strmh->cur_ctrl;

                  UVC_ENTER();

                  if (strmh->running) {
                    UVC_EXIT(UVC_ERROR_BUSY);
                    return UVC_ERROR_BUSY;
                  }

                  strmh->running = 1;
                  strmh->seq = 1;
                  strmh->fid = 0;
                  strmh->pts = 0;
                  strmh->last_scr = 0;

                  frame_desc = uvc_find_frame_desc_stream(strmh, ctrl->bFormatIndex, ctrl->bFrameIndex);


                  //start: uvc_find_frame_desc_stream()-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------

                    /** @internal
                     * @brief Find the descriptor for a specific frame configuration
                     * @param stream_if Stream interface
                     * @param format_id Index of format class descriptor
                     * @param frame_id Index of frame descriptor
                     */
                    static uvc_frame_desc_t *_uvc_find_frame_desc_stream_if(uvc_streaming_interface_t *stream_if,
                        uint16_t format_id, uint16_t frame_id) {
                    
                      uvc_format_desc_t *format = NULL;
                      uvc_frame_desc_t *frame = NULL;

                      DL_FOREACH(stream_if->format_descs, format) {
                        if (format->bFormatIndex == format_id) {
                          DL_FOREACH(format->frame_descs, frame) {
                            if (frame->bFrameIndex == frame_id)
                              return frame;
                          }
                        }
                      }

                      return NULL;
                    }

                    uvc_frame_desc_t *uvc_find_frame_desc_stream(uvc_stream_handle_t *strmh,
                        uint16_t format_id, uint16_t frame_id) {
                      return _uvc_find_frame_desc_stream_if(strmh->stream_if, format_id, frame_id);
                    }
                  //end: uvc_find_frame_desc_stream()-----------------------------------------
                  //-----------------------------------------
                  //-----------------------------------------

                  if (!frame_desc) {
                    ret = UVC_ERROR_INVALID_PARAM;
                    goto fail;
                  }
                  format_desc = frame_desc->parent;

                  strmh->frame_format = uvc_frame_format_for_guid(format_desc->guidFormat);
                  if (strmh->frame_format == UVC_FRAME_FORMAT_UNKNOWN) {
                    ret = UVC_ERROR_NOT_SUPPORTED;
                    goto fail;
                  }

                  // Get the interface that provides the chosen format and frame configuration
                  interface_id = strmh->stream_if->bInterfaceNumber;
                  interface = &strmh->devh->info->config->interface[interface_id];

                  /* A VS interface uses isochronous transfers iff it has multiple altsettings.
                  * (UVC 1.5: 2.4.3. VideoStreaming Interface) */
                  isochronous = interface->num_altsetting > 1;

                  if (isochronous) {
                    /* For isochronous streaming, we choose an appropriate altsetting for the endpoint
                    * and set up several transfers */
                    const struct libusb_interface_descriptor *altsetting = 0;
                    const struct libusb_endpoint_descriptor *endpoint = 0;
                    /* The greatest number of bytes that the device might provide, per packet, in this
                    * configuration */
                    size_t config_bytes_per_packet;
                    /* Number of packets per transfer */
                    size_t packets_per_transfer = 0;
                    /* Size of packet transferable from the chosen endpoint */
                    size_t endpoint_bytes_per_packet = 0;
                    /* Index of the altsetting */
                    int alt_idx, ep_idx;
                    
                    config_bytes_per_packet = strmh->cur_ctrl.dwMaxPayloadTransferSize;

                    /* Go through the altsettings and find one whose packets are at least
                    * as big as our format's maximum per-packet usage. Assume that the
                    * packet sizes are increasing. */
                    for (alt_idx = 0; alt_idx < interface->num_altsetting; alt_idx++) {
                      altsetting = interface->altsetting + alt_idx;
                      endpoint_bytes_per_packet = 0;

                      /* Find the endpoint with the number specified in the VS header */
                      for (ep_idx = 0; ep_idx < altsetting->bNumEndpoints; ep_idx++) {
                        endpoint = altsetting->endpoint + ep_idx;

                        struct libusb_ss_endpoint_companion_descriptor *ep_comp = 0;
                        libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
                        if (ep_comp)
                        {
                          endpoint_bytes_per_packet = ep_comp->wBytesPerInterval;
                          libusb_free_ss_endpoint_companion_descriptor(ep_comp);
                          break;
                        }
                        else
                        {
                          if (endpoint->bEndpointAddress == format_desc->parent->bEndpointAddress) {
                              endpoint_bytes_per_packet = endpoint->wMaxPacketSize;
                            // wMaxPacketSize: [unused:2 (multiplier-1):3 size:11]
                            endpoint_bytes_per_packet = (endpoint_bytes_per_packet & 0x07ff) *
                              (((endpoint_bytes_per_packet >> 11) & 3) + 1);
                            break;
                          }
                        }
                      }

                      if (endpoint_bytes_per_packet >= config_bytes_per_packet) {
                        /* Transfers will be at most one frame long: Divide the maximum frame size
                        * by the size of the endpoint and round up */
                        packets_per_transfer = (ctrl->dwMaxVideoFrameSize +
                                                endpoint_bytes_per_packet - 1) / endpoint_bytes_per_packet;

                        /* But keep a reasonable limit: Otherwise we start dropping data */
                        if (packets_per_transfer > 32)
                          packets_per_transfer = 32;
                        
                        total_transfer_size = packets_per_transfer * endpoint_bytes_per_packet;
                        break;
                      }
                    }

                    /* If we searched through all the altsettings and found nothing usable */
                    if (alt_idx == interface->num_altsetting) {
                      ret = UVC_ERROR_INVALID_MODE;
                      goto fail;
                    }

                    /* Select the altsetting */
                    ret = libusb_set_interface_alt_setting(strmh->devh->usb_devh,
                                                          altsetting->bInterfaceNumber,
                                                          altsetting->bAlternateSetting);
                    if (ret != UVC_SUCCESS) {
                      UVC_DEBUG("libusb_set_interface_alt_setting failed");
                      goto fail;
                    }

                    /* Set up the transfers */
                    for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; ++transfer_id) {
                      transfer = libusb_alloc_transfer(packets_per_transfer);
                      strmh->transfers[transfer_id] = transfer;      
                      strmh->transfer_bufs[transfer_id] = malloc(total_transfer_size);

                      libusb_fill_iso_transfer(
                        transfer, strmh->devh->usb_devh, format_desc->parent->bEndpointAddress,
                        strmh->transfer_bufs[transfer_id],
                        total_transfer_size, packets_per_transfer, _uvc_stream_callback, (void*) strmh, 5000);

                          //end: _uvc_stream_callback()-----------------------------------------
                          //-----------------------------------------
                          //-----------------------------------------
                          /** @internal
                           * @brief Stream transfer callback
                           *
                           * Processes stream, places frames into buffer, signals listeners
                           * (such as user callback thread and any polling thread) on new frame
                           *
                           * @param transfer Active transfer
                           */
                          void LIBUSB_CALL _uvc_stream_callback(struct libusb_transfer *transfer) {
                            uvc_stream_handle_t *strmh = transfer->user_data;

                            int resubmit = 1;

                            switch (transfer->status) {
                            case LIBUSB_TRANSFER_COMPLETED:
                              if (transfer->num_iso_packets == 0) {
                                /* This is a bulk mode transfer, so it just has one payload transfer */
                                _uvc_process_payload(strmh, transfer->buffer, transfer->actual_length);
                              } else {
                                /* This is an isochronous mode transfer, so each packet has a payload transfer */
                                int packet_id;

                                for (packet_id = 0; packet_id < transfer->num_iso_packets; ++packet_id) {
                                  uint8_t *pktbuf;
                                  struct libusb_iso_packet_descriptor *pkt;

                                  pkt = transfer->iso_packet_desc + packet_id;

                                  if (pkt->status != 0) {
                                    UVC_DEBUG("bad packet (isochronous transfer); status: %d", pkt->status);
                                    continue;
                                  }

                                  pktbuf = libusb_get_iso_packet_buffer_simple(transfer, packet_id);

                                  _uvc_process_payload(strmh, pktbuf, pkt->actual_length);

                                }
                              }
                              break;
                            case LIBUSB_TRANSFER_CANCELLED: 
                            case LIBUSB_TRANSFER_ERROR:
                            case LIBUSB_TRANSFER_NO_DEVICE: {
                              int i;
                              UVC_DEBUG("not retrying transfer, status = %d", transfer->status);
                              pthread_mutex_lock(&strmh->cb_mutex);

                              /* Mark transfer as deleted. */
                              for(i=0; i < LIBUVC_NUM_TRANSFER_BUFS; i++) {
                                if(strmh->transfers[i] == transfer) {
                                  UVC_DEBUG("Freeing transfer %d (%p)", i, transfer);
                                  free(transfer->buffer);
                                  libusb_free_transfer(transfer);
                                  strmh->transfers[i] = NULL;
                                  break;
                                }
                              }
                              if(i == LIBUVC_NUM_TRANSFER_BUFS ) {
                                UVC_DEBUG("transfer %p not found; not freeing!", transfer);
                              }

                              resubmit = 0;

                              pthread_cond_broadcast(&strmh->cb_cond);
                              pthread_mutex_unlock(&strmh->cb_mutex);

                              break;
                            }
                            case LIBUSB_TRANSFER_TIMED_OUT:
                            case LIBUSB_TRANSFER_STALL:
                            case LIBUSB_TRANSFER_OVERFLOW:
                              UVC_DEBUG("retrying transfer, status = %d", transfer->status);
                              break;
                            }
                            
                            if ( resubmit ) {
                              if ( strmh->running ) {
                                int libusbRet = libusb_submit_transfer(transfer);
                                if (libusbRet < 0)
                                {
                                  int i;
                                  pthread_mutex_lock(&strmh->cb_mutex);

                                  /* Mark transfer as deleted. */
                                  for (i = 0; i < LIBUVC_NUM_TRANSFER_BUFS; i++) {
                                    if (strmh->transfers[i] == transfer) {
                                      UVC_DEBUG("Freeing failed transfer %d (%p)", i, transfer);
                                      free(transfer->buffer);
                                      libusb_free_transfer(transfer);
                                      strmh->transfers[i] = NULL;
                                      break;
                                    }
                                  }
                                  if (i == LIBUVC_NUM_TRANSFER_BUFS) {
                                    UVC_DEBUG("failed transfer %p not found; not freeing!", transfer);
                                  }

                                  pthread_cond_broadcast(&strmh->cb_cond);
                                  pthread_mutex_unlock(&strmh->cb_mutex);
                                }
                              } else {
                                int i;
                                pthread_mutex_lock(&strmh->cb_mutex);

                                /* Mark transfer as deleted. */
                                for(i=0; i < LIBUVC_NUM_TRANSFER_BUFS; i++) {
                                  if(strmh->transfers[i] == transfer) {
                                    UVC_DEBUG("Freeing orphan transfer %d (%p)", i, transfer);
                                    free(transfer->buffer);
                                    libusb_free_transfer(transfer);
                                    strmh->transfers[i] = NULL;
                                    break;
                                  }
                                }
                                if(i == LIBUVC_NUM_TRANSFER_BUFS ) {
                                  UVC_DEBUG("orphan transfer %p not found; not freeing!", transfer);
                                }

                                pthread_cond_broadcast(&strmh->cb_cond);
                                pthread_mutex_unlock(&strmh->cb_mutex);
                              }
                            }
                          }
                          //end: _uvc_stream_callback()-----------------------------------------
                          //-----------------------------------------
                          //-----------------------------------------


                      libusb_set_iso_packet_lengths(transfer, endpoint_bytes_per_packet);
                    }
                  } else {
                    for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS;
                        ++transfer_id) {
                      transfer = libusb_alloc_transfer(0);
                      strmh->transfers[transfer_id] = transfer;
                      strmh->transfer_bufs[transfer_id] = malloc (
                          strmh->cur_ctrl.dwMaxPayloadTransferSize );
                      libusb_fill_bulk_transfer ( transfer, strmh->devh->usb_devh,
                          format_desc->parent->bEndpointAddress,
                          strmh->transfer_bufs[transfer_id],
                          strmh->cur_ctrl.dwMaxPayloadTransferSize, _uvc_stream_callback,
                          ( void* ) strmh, 5000 );
                    }
                  }

                  strmh->user_cb = cb;
                  strmh->user_ptr = user_ptr;

                  /* If the user wants it, set up a thread that calls the user's function
                  * with the contents of each frame.
                  */
                  if (cb) {
                    pthread_create(&strmh->cb_thread, NULL, _uvc_user_caller, (void*) strmh);
                  }

                  for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS;
                      transfer_id++) {
                    ret = libusb_submit_transfer(strmh->transfers[transfer_id]);
                    if (ret != UVC_SUCCESS) {
                      UVC_DEBUG("libusb_submit_transfer failed: %d",ret);
                      break;
                    }
                  }

                  if ( ret != UVC_SUCCESS && transfer_id >= 0 ) {
                    for ( ; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; transfer_id++) {
                      free ( strmh->transfers[transfer_id]->buffer );
                      libusb_free_transfer ( strmh->transfers[transfer_id]);
                      strmh->transfers[transfer_id] = 0;
                    }
                    ret = UVC_SUCCESS;
                  }

                  UVC_EXIT(ret);
                  return ret;
                fail:
                  strmh->running = 0;
                  UVC_EXIT(ret);
                  return ret;
                }

              //end: uvc_stream_start()-----------------------------------------
              //-----------------------------------------
              //-----------------------------------------   

              if (ret != UVC_SUCCESS) {
                uvc_stream_close(strmh);
                return ret;
              }

              return UVC_SUCCESS;
            }

        //end: uvc_start_streaming()-----------------------------------------
        //-----------------------------------------
        //-----------------------------------------
        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          puts("Streaming...");

          /* enable auto exposure - see uvc_set_ae_mode documentation */
          puts("Enabling auto exposure ...");
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            puts(" ... enabled auto exposure");
          } else if (res == UVC_ERROR_PIPE) {
            /* this error indicates that the camera does not support the full AE mode;
             * try again, using aperture priority mode (fixed aperture, variable exposure time) */
            puts(" ... full AE not supported, trying aperture priority mode");
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture priority mode");
            } else {
              puts(" ... enabled aperture priority auto exposure mode");
            }
          } else {
            uvc_perror(res, " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }

          sleep(10); /* stream for 10 seconds */

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          puts("Done streaming.");
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      puts("Device closed");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  puts("UVC exited");

  return 0;
}






//======================================================
//======================================================
//======================================================



//int main(int argc, char **argv) {

  //start: uvc_init()
    //*** libusb_init()
  //end:   uvc_init()

  //start: uvc_find_device()
    //start: uvc_get_device_list()
      //*** libusb_get_device_list
      //*** libusb_get_config_descriptor
    //end:   uvc_get_device_list()

    //while (!found_dev && (test_dev = list[dev_idx++]) != NULL)
      //start: uvc_get_device_descriptor()
        //*** libusb_get_device_descriptor()
        //*** libusb_open()
        //*** libusb_get_string_descriptor_ascii()
      //end: uvc_get_device_descriptor()
  //end:   uvc_find_device()


  //start: uvc_open()
    //*** libusb_open()
    //start: uvc_open_internal()
      //*** libusb_get_device_descriptor()
      //*** libusb_fill_interrupt_transfer()
      //*** libusb_submit_transfer()
      //start: uvc_start_handler_thread()
        //start: _uvc_handle_events
          // while (!ctx->kill_handler_thread)
          //***   libusb_handle_events_completed(ctx->usb_ctx, &ctx->kill_handler_thread);      // NO
          // return NULL;
        //end:   _uvc_handle_events
      //end:  uvc_start_handler_thread()


    //end:   uvc_open_internal()
  //end:   uvc_open()

  //start: uvc_print_diag()
  //end:   uvc_print_diag()

  //start: uvc_get_format_descs()  
  //end:   uvc_get_format_descs()

  //start: uvc_get_stream_ctrl_format_size()
  //end:   uvc_get_stream_ctrl_format_size()

  //start: uvc_print_stream_ctrl()
  //end:   uvc_print_stream_ctrl()


  //start: uvc_start_streaming()

    //start: uvc_stream_open_ctrl()
      //start: _uvc_get_stream_by_interface()
      //end:   _uvc_get_stream_by_interface()

      //start:_uvc_get_stream_if()
      //end:  _uvc_get_stream_if()

      //start: uvc_claim_if()
        //*** libusb_detach_kernel_driver()
        //*** libusb_claim_interface()
      //end:   uvc_claim_if()

      //start: uvc_stream_ctrl()
        //start: uvc_query_stream_ctrl()
          //start: uvc_find_frame_desc()
            //start: _uvc_find_frame_desc_stream_if()
            //stop:  _uvc_find_frame_desc_stream_if()
          //end: uvc_find_frame_desc() 
        //end: uvc_query_stream_ctrl()
      //end: uvc_stream_ctrl()  
    //end: uvc_stream_open_ctrl()

    //start: uvc_stream_start()
      //start: uvc_find_frame_desc_stream()
        // isochronous = interface->num_altsetting > 1;
        // if (isochronous) {
          // for (alt_idx = 0; alt_idx < interface->num_altsetting; alt_idx++) {
            // for (ep_idx = 0; ep_idx < altsetting->bNumEndpoints; ep_idx++) {
            //  endpoint = altsetting->endpoint + ep_idx;
            //  endpoint_bytes_per_packet = ep_comp->wBytesPerInterval
            //***  libusb_get_ss_endpoint_companion_descriptor()
            // }
          // }
          // libusb_set_interface_alt_setting
          // for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; ++transfer_id) {
            //*** libusb_alloc_transfer()
            //*** libusb_fill_iso_transfer()
              //  _uvc_stream_callback()
                //*** libusb_get_iso_packet_buffer_simple()
            //*** libusb_set_iso_packet_lengths()
          // }
        // }
        // for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS; transfer_id++) {
          //*** libusb_submit_transfer()
        // }
        // *** libusb_free_transfer()
      //end:   uvc_find_frame_desc_stream()
    //end:   uvc_stream_start()

  //end: uvc_start_streaming() 

//}



//======================================================
//======================================================
//======================================================

// /home/andriy/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/libuvc/src/init.c

/**
 * @internal
 * @brief Spawns a handler thread for the context
 * @ingroup init
 *
 * This should be called at the end of a successful uvc_open if no devices
 * are already open (and being handled).
 */
void uvc_start_handler_thread(uvc_context_t *ctx) {
  if (ctx->own_usb_ctx)
    pthread_create(&ctx->handler_thread, NULL, _uvc_handle_events, (void*) ctx);
}


// /home/andriy/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/libuvc/src/init.c

/** @internal
 * @brief Event handler thread
 * There's one of these per UVC context.
 * @todo We shouldn't run this if we don't own the USB context
 */
void *_uvc_handle_events(void *arg) {
  uvc_context_t *ctx = (uvc_context_t *) arg;

  while (!ctx->kill_handler_thread)
    libusb_handle_events_completed(ctx->usb_ctx, &ctx->kill_handler_thread);
  return NULL;
}




//======================================================
//======================================================
//======================================================

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;
  // sudo ./example 2>/dev/null 1>video.yuv
  // write(1, frame->data, frame->data_bytes);
}



res = uvc_start_streaming(devh, &ctrl, cb, (void *) 12345, 0);




/** Begin streaming video from the camera into the callback function.
 * @ingroup streaming
 *
 * @param devh UVC device
 * @param ctrl Control block, processed using {uvc_probe_stream_ctrl} or
 *             {uvc_get_stream_ctrl_format_size}
 * @param cb   User callback function. See {uvc_frame_callback_t} for restrictions.
 * @param flags Stream setup flags, currently undefined. Set this to zero. The lower bit
 * is reserved for backward compatibility.
 */
uvc_error_t uvc_start_streaming(
    uvc_device_handle_t *devh,
    uvc_stream_ctrl_t *ctrl,
    uvc_frame_callback_t *cb,
    void *user_ptr,
    uint8_t flags
) {
  uvc_error_t ret;
  uvc_stream_handle_t *strmh;

  ret = uvc_stream_open_ctrl(devh, &strmh, ctrl);
  if (ret != UVC_SUCCESS)
    return ret;

  ret = uvc_stream_start(strmh, cb, user_ptr, flags);
  if (ret != UVC_SUCCESS) {
    uvc_stream_close(strmh);
    return ret;
  }

  return UVC_SUCCESS;
}



uvc_error_t uvc_stream_start(
    uvc_stream_handle_t *strmh,
    uvc_frame_callback_t *cb,
    void *user_ptr,
    uint8_t flags
) {

  strmh->user_cb = cb;
  strmh->user_ptr = user_ptr;

  /* If the user wants it, set up a thread that calls the user's function
   * with the contents of each frame.  !!!!!!!!!!!!!! Тут есть различие между Qemu ????????????
   */
  if (cb) {
    pthread_create(&strmh->cb_thread, NULL, _uvc_user_caller, (void*) strmh);
  }

  for (transfer_id = 0; transfer_id < LIBUVC_NUM_TRANSFER_BUFS;
      transfer_id++) {
    ret = libusb_submit_transfer(strmh->transfers[transfer_id]);
    if (ret != UVC_SUCCESS) {
      UVC_DEBUG("libusb_submit_transfer failed: %d",ret);
      break;
    }
  }


}



/** @internal
 * @brief User callback runner thread
 * @note There should be at most one of these per currently streaming device
 * @param arg Device handle
 */
void *_uvc_user_caller(void *arg) {
  uvc_stream_handle_t *strmh = (uvc_stream_handle_t *) arg;

  uint32_t last_seq = 0;

  do {
    pthread_mutex_lock(&strmh->cb_mutex);

    while (strmh->running && last_seq == strmh->hold_seq) {
      pthread_cond_wait(&strmh->cb_cond, &strmh->cb_mutex);
    }

    if (!strmh->running) {
      pthread_mutex_unlock(&strmh->cb_mutex);
      break;
    }
    
    last_seq = strmh->hold_seq;
    _uvc_populate_frame(strmh);
    
    pthread_mutex_unlock(&strmh->cb_mutex);
    
    strmh->user_cb(&strmh->frame, strmh->user_ptr);
  } while(1);

  return NULL; // return value ignored
}






/** @internal
 * @brief Populate the fields of a frame to be handed to user code
 * must be called with stream cb lock held!
 */
void _uvc_populate_frame(uvc_stream_handle_t *strmh) {
  uvc_frame_t *frame = &strmh->frame;
  uvc_frame_desc_t *frame_desc;

  /** @todo this stuff that hits the main config cache should really happen
   * in start() so that only one thread hits these data. all of this stuff
   * is going to be reopen_on_change anyway
   */

  frame_desc = uvc_find_frame_desc(strmh->devh, strmh->cur_ctrl.bFormatIndex,
				   strmh->cur_ctrl.bFrameIndex);

  frame->frame_format = strmh->frame_format;
  
  frame->width = frame_desc->wWidth;
  frame->height = frame_desc->wHeight;
  
  switch (frame->frame_format) {
  case UVC_FRAME_FORMAT_BGR:
    frame->step = frame->width * 3;
    break;
  case UVC_FRAME_FORMAT_YUYV:
    frame->step = frame->width * 2;
    break;
  case UVC_FRAME_FORMAT_NV12:
    frame->step = frame->width;
    break;
  case UVC_FRAME_FORMAT_MJPEG:
    frame->step = 0;
    break;
  case UVC_FRAME_FORMAT_H264:
    frame->step = 0;
    break;
  default:
    frame->step = 0;
    break;
  }

  frame->sequence = strmh->hold_seq;
  frame->capture_time_finished = strmh->capture_time_finished;

  /* copy the image data from the hold buffer to the frame (unnecessary extra buf?) */
  if (frame->data_bytes < strmh->hold_bytes) {
    frame->data = realloc(frame->data, strmh->hold_bytes);
  }
  frame->data_bytes = strmh->hold_bytes;
  memcpy(frame->data, strmh->holdbuf, frame->data_bytes);

  if (strmh->meta_hold_bytes > 0)
  {
      if (frame->metadata_bytes < strmh->meta_hold_bytes)
      {
          frame->metadata = realloc(frame->metadata, strmh->meta_hold_bytes);
      }
      frame->metadata_bytes = strmh->meta_hold_bytes;
      memcpy(frame->metadata, strmh->meta_holdbuf, frame->metadata_bytes);
  }
}



//======================================================
//======================================================
//======================================================


uvc_device_t *dev;
uvc_find_device(ctx, &dev, 0, 0, NULL);

uvc_error_t uvc_find_device(uvc_context_t *ctx, uvc_device_t **dev, int vid, int pid, const char *sn)




//======================================================
//======================================================
//======================================================


uvc_error_t uvc_open(uvc_device_t *dev, uvc_device_handle_t **devh) {

  // Тут я просто взял, и всю хуйню заменил на:
  // usb_devh = libusb_open_device_with_vid_pid(NULL, 0x046d, 0x0825)

}


static uvc_error_t uvc_open_internal(uvc_device_t *dev,struct libusb_device_handle *usb_devh, uvc_device_handle_t **devh) {
  
  uvc_error_t ret;
  uvc_device_handle_t *internal_devh;
  struct libusb_device_descriptor desc;



  UVC_ENTER();

  uvc_ref_device(dev);

  internal_devh = calloc(1, sizeof(*internal_devh));
  internal_devh->dev = dev;
  internal_devh->usb_devh = usb_devh;


  ret = uvc_get_device_info(internal_devh, &(internal_devh->info));

  dgnetP_deviceC("device.c ::: uvc_open_internal: uvc_device_handle_t: internal_devh->info->ctrl_if.bInterfaceNumber: %d \n", internal_devh->info->ctrl_if.bInterfaceNumber);
  dgnetP_deviceC("device.c ::: uvc_open_internal: uvc_device_handle_t: internal_devh->info->ctrl_if.bcdUVC: %d \n", internal_devh->info->ctrl_if.bcdUVC);
  dgnetP_deviceC("device.c ::: uvc_open_internal: uvc_device_handle_t: internal_devh->info->ctrl_if.dwClockFrequency: %d \n", internal_devh->info->ctrl_if.dwClockFrequency);
  dgnetP_deviceC("device.c ::: uvc_open_internal: uvc_device_handle_t: internal_devh->info->ctrl_if.bEndpointAddress: %d \n", internal_devh->info->ctrl_if.bEndpointAddress);


  et = uvc_claim_if(internal_devh, internal_devh->info->ctrl_if.bInterfaceNumber);


}

uvc_error_t uvc_get_device_info(uvc_device_handle_t *devh,
				uvc_device_info_t **info) {
  uvc_error_t ret;
  uvc_device_info_t *internal_info;

  UVC_ENTER();

  internal_info = calloc(1, sizeof(*internal_info));
  if (!internal_info) {
    UVC_EXIT(UVC_ERROR_NO_MEM);
    return UVC_ERROR_NO_MEM;
  }

  if (libusb_get_config_descriptor(devh->dev->usb_dev,
				   0,
				   &(internal_info->config)) != 0) {
    free(internal_info);
    UVC_EXIT(UVC_ERROR_IO);
    return UVC_ERROR_IO;
  }

  ret = uvc_scan_control(devh, internal_info);
  if (ret != UVC_SUCCESS) {
    uvc_free_device_info(internal_info);
    UVC_EXIT(ret);
    return ret;
  }

  *info = internal_info;

  UVC_EXIT(ret);
  return ret;
}






//======================================================
//======================================================
//======================================================






































//======================================================
//===================LIBUVC=============================
//======================================================



//*** libusb_init()



