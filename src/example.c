// andriy@andriyK:~/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/results:
// $ ffmpeg -f rawvideo -pix_fmt yuyv422 -s:v 640x480 -r 15 -i ./out.yuv output.avi
// ffmpeg version 4.2.4-1ubuntu0.1 Copyright (c) 2000-2020 the FFmpeg developers
//   built with gcc 9 (Ubuntu 9.3.0-10ubuntu2)
//   configuration: --prefix=/usr --extra-version=1ubuntu0.1 --toolchain=hardened --libdir=/usr/lib/x86_64-linux-gnu --incdir=/usr/include/x86_64-linux-gnu --arch=amd64 --enable-gpl --disable-stripping --enable-avresample --disable-filter=resample --enable-avisynth --enable-gnutls --enable-ladspa --enable-libaom --enable-libass --enable-libbluray --enable-libbs2b --enable-libcaca --enable-libcdio --enable-libcodec2 --enable-libflite --enable-libfontconfig --enable-libfreetype --enable-libfribidi --enable-libgme --enable-libgsm --enable-libjack --enable-libmp3lame --enable-libmysofa --enable-libopenjpeg --enable-libopenmpt --enable-libopus --enable-libpulse --enable-librsvg --enable-librubberband --enable-libshine --enable-libsnappy --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libtwolame --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwavpack --enable-libwebp --enable-libx265 --enable-libxml2 --enable-libxvid --enable-libzmq --enable-libzvbi --enable-lv2 --enable-omx --enable-openal --enable-opencl --enable-opengl --enable-sdl2 --enable-libdc1394 --enable-libdrm --enable-libiec61883 --enable-nvenc --enable-chromaprint --enable-frei0r --enable-libx264 --enable-shared
//   libavutil      56. 31.100 / 56. 31.100
//   libavcodec     58. 54.100 / 58. 54.100
//   libavformat    58. 29.100 / 58. 29.100
//   libavdevice    58.  8.100 / 58.  8.100
//   libavfilter     7. 57.100 /  7. 57.100
//   libavresample   4.  0.  0 /  4.  0.  0
//   libswscale      5.  5.100 /  5.  5.100
//   libswresample   3.  5.100 /  3.  5.100
//   libpostproc    55.  5.100 / 55.  5.100
// [rawvideo @ 0x55f864540700] Estimating duration from bitrate, this may be inaccurate
// Input #0, rawvideo, from './out.yuv':
//   Duration: 00:00:09.87, start: 0.000000, bitrate: 73727 kb/s
//     Stream #0:0: Video: rawvideo (YUY2 / 0x32595559), yuyv422, 640x480, 73728 kb/s, 15 tbr, 15 tbn, 15 tbc
// Stream mapping:
//   Stream #0:0 -> #0:0 (rawvideo (native) -> mpeg4 (native))
// Press [q] to stop, [?] for help
// Output #0, avi, to 'output.avi':
//   Metadata:
//     ISFT            : Lavf58.29.100
//     Stream #0:0: Video: mpeg4 (FMP4 / 0x34504D46), yuv420p, 640x480, q=2-31, 200 kb/s, 15 fps, 15 tbn, 15 tbc
//     Metadata:
//       encoder         : Lavc58.54.100 mpeg4
//     Side data:
//       cpb: bitrate max/min/avg: 0/0/200000 buffer size: 0 vbv_delay: -1
// frame=  148 fps=0.0 q=31.0 Lsize=     422kB time=00:00:09.86 bitrate= 350.1kbits/s speed= 126x    
// video:413kB audio:0kB subtitle:0kB other streams:0kB global headers:0kB muxing overhead: 2.202416%
// andriy@andriyK:~/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/results:
// $ 


// But sometimes the following error appears.
// Does it have to do with how the stream ends?:


// andriy@andriyK:~/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/results:
// $ ffmpeg -f rawvideo -pix_fmt yuyv422 -s:v 640x480 -r 15 -i ./out.yuv output.avi
// ffmpeg version 4.2.4-1ubuntu0.1 Copyright (c) 2000-2020 the FFmpeg developers
//   built with gcc 9 (Ubuntu 9.3.0-10ubuntu2)
//   configuration: --prefix=/usr --extra-version=1ubuntu0.1 --toolchain=hardened --libdir=/usr/lib/x86_64-linux-gnu --incdir=/usr/include/x86_64-linux-gnu --arch=amd64 --enable-gpl --disable-stripping --enable-avresample --disable-filter=resample --enable-avisynth --enable-gnutls --enable-ladspa --enable-libaom --enable-libass --enable-libbluray --enable-libbs2b --enable-libcaca --enable-libcdio --enable-libcodec2 --enable-libflite --enable-libfontconfig --enable-libfreetype --enable-libfribidi --enable-libgme --enable-libgsm --enable-libjack --enable-libmp3lame --enable-libmysofa --enable-libopenjpeg --enable-libopenmpt --enable-libopus --enable-libpulse --enable-librsvg --enable-librubberband --enable-libshine --enable-libsnappy --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libtwolame --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwavpack --enable-libwebp --enable-libx265 --enable-libxml2 --enable-libxvid --enable-libzmq --enable-libzvbi --enable-lv2 --enable-omx --enable-openal --enable-opencl --enable-opengl --enable-sdl2 --enable-libdc1394 --enable-libdrm --enable-libiec61883 --enable-nvenc --enable-chromaprint --enable-frei0r --enable-libx264 --enable-shared
//   libavutil      56. 31.100 / 56. 31.100
//   libavcodec     58. 54.100 / 58. 54.100
//   libavformat    58. 29.100 / 58. 29.100
//   libavdevice    58.  8.100 / 58.  8.100
//   libavfilter     7. 57.100 /  7. 57.100
//   libavresample   4.  0.  0 /  4.  0.  0
//   libswscale      5.  5.100 /  5.  5.100
//   libswresample   3.  5.100 /  3.  5.100
//   libpostproc    55.  5.100 / 55.  5.100
// [rawvideo @ 0x562bb1b68700] Estimating duration from bitrate, this may be inaccurate
// Input #0, rawvideo, from './out.yuv':
//   Duration: 00:00:09.87, start: 0.000000, bitrate: 73727 kb/s
//     Stream #0:0: Video: rawvideo (YUY2 / 0x32595559), yuyv422, 640x480, 73728 kb/s, 15 tbr, 15 tbn, 15 tbc
// Stream mapping:
//   Stream #0:0 -> #0:0 (rawvideo (native) -> mpeg4 (native))
// Press [q] to stop, [?] for help
// Output #0, avi, to 'output.avi':
//   Metadata:
//     ISFT            : Lavf58.29.100
//     Stream #0:0: Video: mpeg4 (FMP4 / 0x34504D46), yuv420p, 640x480, q=2-31, 200 kb/s, 15 fps, 15 tbn, 15 tbc
//     Metadata:
//       encoder         : Lavc58.54.100 mpeg4
//     Side data:
//       cpb: bitrate max/min/avg: 0/0/200000 buffer size: 0 vbv_delay: -1
// ./out.yuv: corrupt input packet in stream 0
// [rawvideo @ 0x562bb1b74740] Invalid buffer size, packet size 614396 < expected frame_size 614400
// Error while decoding stream #0:0: Invalid argument
// frame=  147 fps=0.0 q=14.6 Lsize=     392kB time=00:00:09.80 bitrate= 327.3kbits/s speed= 132x    
// video:383kB audio:0kB subtitle:0kB other streams:0kB global headers:0kB muxing overhead: 2.368091%
// andriy@andriyK:~/Documents/DGC/Reports/QEMU/Logi_Rally_Camera/results:
// $ 

#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>

#include <stdarg.h>
void dgnetP_exampleC(char *format, ...){

    // FILE * pFile;
    // pFile = fopen ("/home/dgnet/build/results/libuvc_out.txt","a");

    // va_list args;
    // va_start(args, format);
    // vfprintf(pFile, format, args);
    // va_end(args);  
    // fclose(pFile);
}
//dgnetP_exampleC("example.c ::: function_name() ::: %s \n", "message");


void dgnetP_exampleC_libusb(char *format, ...){

    FILE * pFile;
    pFile = fopen ("/home/dgnet/build/results/libuvc_libusb_out.txt","a");

    va_list args;
    va_start(args, format);
    vfprintf(pFile, format, args);
    va_end(args);  
    fclose(pFile);
}
//dgnetP_exampleC_libusb("stream.c ::: function_name() ::: %s \n", "message");



int fdnum_yuv;
int fdnum_bgr;

/* This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
void cb(uvc_frame_t *frame, void *ptr) {
  uvc_frame_t *bgr;
  uvc_error_t ret;

  dgnetP_exampleC("example.c ::: cb(): %s \n", "0");

  dgnetP_exampleC("example.c ::: cb(): frame->data_bytes %d \n", frame->data_bytes);

  dgnetP_exampleC_libusb("example.c ::: cb(): frame->data_bytes %d \n", frame->data_bytes);

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

  //printf("callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %d\n", frame->frame_format, frame->width, frame->height, frame->data_bytes, (int) ptr);
  dgnetP_exampleC("example.c ::: cb() ::: callback! frame_format = %d, width = %d, height = %d, length = %lu, ptr = %d\n", frame->frame_format, frame->width, frame->height, frame->data_bytes, (int) ptr);

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
    // ret = uvc_any2bgr(frame, bgr);
    // if (ret) {
    //   uvc_perror(ret, "uvc_any2bgr");
    //   uvc_free_frame(bgr);
    //   return;
    // }
    // write(fdnum_bgr, bgr->data, bgr->data_bytes);
    break;
  default:
    break;
  }

  if (frame->sequence % 30 == 0) {
    //printf(" * got image %u\n",  frame->sequence);
    dgnetP_exampleC("example.c ::: cb() ::: * got image %u\n",  frame->sequence);
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

  dgnetP_exampleC("example.c ::: cb(): %s \n", "999");

  uvc_free_frame(bgr);
}

int main(int argc, char **argv) {
  uvc_context_t *ctx;
  uvc_device_t *dev;
  uvc_device_handle_t *devh;
  uvc_stream_ctrl_t ctrl;
  uvc_error_t res;

  FILE* h_yuv = fopen("/home/dgnet/build/results/out.yuv", "a+");
  fdnum_yuv = fileno(h_yuv);

  FILE* h_bgr = fopen("/home/dgnet/build/results/out.bgr", "a+");
  fdnum_bgr = fileno(h_bgr);


  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */

  dgnetP_exampleC("example.c ::: main() ::: %s \n", "start: uvc_init()"); 
  res = uvc_init(&ctx, NULL);
  dgnetP_exampleC("example.c ::: main() ::: %s \n", "end: uvc_init()"); 

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    return res;
  }

  //puts("UVC initialized");
  dgnetP_exampleC("example.c ::: UVC initialized \n");

  /* Locates the first attached UVC device, stores in dev */
  dgnetP_exampleC("example.c ::: main() ::: %s \n", "start: uvc_find_device()");
  res = uvc_find_device(
      ctx, &dev,
      0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */
  //dgnetP_exampleC("example.c ::: main() ::: %s, %d \n", "end: uvc_find_device()", dev->ref);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
  } else {
    //puts("Device found");
    dgnetP_exampleC("example.c ::: Devide found\n");

    dgnetP_exampleC("example.c ::: main() ::: %s \n", "start: uvc_open()");
    /* Try to open the device: requires exclusive access */
    res = uvc_open(dev, &devh);
    dgnetP_exampleC("example.c ::: main() ::: %s \n", "end: uvc_open()");



    if (res < 0) {
      uvc_perror(res, "uvc_open"); /* unable to open device */
    } else {
      //puts("Device opened");
      dgnetP_exampleC("example.c ::: main() ::: %s \n", "Device opened");


      /* Print out a message containing all the information that libuvc
       * knows about the device */
      uvc_print_diag(devh, stderr);

      const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
      const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;      // 640x480

      // Rally
      // frame_desc = frame_desc->next;  // 160x120
      // frame_desc = frame_desc->next;  // 176x144
      // frame_desc = frame_desc->next;  // 320x180
      // frame_desc = frame_desc->next;  // 320x240
      // frame_desc = frame_desc->next;  // 352x288
      // frame_desc = frame_desc->next;  // 480x270
      // frame_desc = frame_desc->next;  // 640x360
      // frame_desc = frame_desc->next;  // 800x448
      // frame_desc = frame_desc->next;  // 800x600
      // frame_desc = frame_desc->next;  // 848x480
      // frame_desc = frame_desc->next;  // 960x540
      // frame_desc = frame_desc->next;  // 1024x576
      // frame_desc = frame_desc->next;  // 1280x720
      // frame_desc = frame_desc->next;  // 1600x896
      // frame_desc = frame_desc->next;  // 1920x1080

      // Mine
      // frame_desc = frame_desc->next; // 160x120
      // frame_desc = frame_desc->next; // 176x144
      // frame_desc = frame_desc->next; // 320x176
      // frame_desc = frame_desc->next; // 320x240
      // frame_desc = frame_desc->next; // 352x288
      // frame_desc = frame_desc->next; // 432x240
      // frame_desc = frame_desc->next; // 544x288
      // frame_desc = frame_desc->next; // 640x360
      // frame_desc = frame_desc->next; // 752x416
      // frame_desc = frame_desc->next; // 800x448
      // frame_desc = frame_desc->next; // 800x600
      // frame_desc = frame_desc->next; // 864x480
      // frame_desc = frame_desc->next; // 960x544
      // frame_desc = frame_desc->next; // 960x720
      // frame_desc = frame_desc->next; // 1024x576
      // frame_desc = frame_desc->next; // 1184x656
      // frame_desc = frame_desc->next; // 1280x720
      // frame_desc = frame_desc->next; // 1280x960




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

      //printf("\nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat, width, height, fps);
      dgnetP_exampleC("example.c ::: main() ::: \nFirst format: (%4s) %dx%d %dfps\n", format_desc->fourccFormat, width, height, fps);

      /* Try to negotiate first stream profile */
      res = uvc_get_stream_ctrl_format_size(
          devh, &ctrl, /* result stored in ctrl */
          frame_format,
          width, height, fps /* width, height, fps */
      );

      /* Print out the result */
      uvc_print_stream_ctrl(&ctrl, stderr);

      if (res < 0) {
        uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
      } else {
        /* Start the video stream. The library will call user function cb:
         *   cb(frame, (void *) 12345)
         */
        res = uvc_start_streaming(devh, &ctrl, cb, (void *) 12345, 0);

        if (res < 0) {
          uvc_perror(res, "start_streaming"); /* unable to start stream */
        } else {
          //puts("Streaming...");
          dgnetP_exampleC("example.c ::: main() ::: Streaming...\n");

          /* enable auto exposure - see uvc_set_ae_mode documentation */
          //puts("Enabling auto exposure ...");
          dgnetP_exampleC("example.c ::: main() ::: Enabling auto exposure ...\n");
          const uint8_t UVC_AUTO_EXPOSURE_MODE_AUTO = 2;
          res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_AUTO);
          if (res == UVC_SUCCESS) {
            puts(" ... enabled auto exposure");
          } else if (res == UVC_ERROR_PIPE) {
            /* this error indicates that the camera does not support the full AE mode;
             * try again, using aperture priority mode (fixed aperture, variable exposure time) */
           //puts(" ... full AE not supported, trying aperture priority mode");
           dgnetP_exampleC("example.c ::: main() :::  ... full AE not supported, trying aperture priority mode \n");
            const uint8_t UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY = 8;
            res = uvc_set_ae_mode(devh, UVC_AUTO_EXPOSURE_MODE_APERTURE_PRIORITY);
            if (res < 0) {
              uvc_perror(res, " ... uvc_set_ae_mode failed to enable aperture priority mode");
            } else {
              //puts(" ... enabled aperture priority auto exposure mode");
              dgnetP_exampleC("example.c ::: main() :::  ... full AE not supported, trying aperture priority mode \n");
            }
          } else {
            uvc_perror(res, " ... uvc_set_ae_mode failed to enable auto exposure mode");
          }

          sleep(10); /* stream for 10 seconds */

          /* End the stream. Blocks until last callback is serviced */
          uvc_stop_streaming(devh);
          //puts("Done streaming.");
          dgnetP_exampleC("example.c ::: main() ::: Done streaming.\n");
          
        }
      }

      /* Release our handle on the device */
      uvc_close(devh);
      //puts("Device closed");
      dgnetP_exampleC("example.c ::: main() ::: Device closed \n");
    }

    /* Release the device descriptor */
    uvc_unref_device(dev);
  }

  /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
  uvc_exit(ctx);
  //puts("UVC exited");
  dgnetP_exampleC("example.c ::: main() ::: UVC exited \n");

  return 0;
}

