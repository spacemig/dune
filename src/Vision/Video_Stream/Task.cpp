//***************************************************************************
// Copyright 2007-2015 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Pedro Gonçalves                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <iostream>

//OpenCV headers
#include <opencv2/opencv.hpp>

//Enable(1) / disable(0) support for Raspicam
#define raspicam_on 0

//RaspiCAM headers
#if raspicam_on == 1
//RaspiCAM headers
#include "RaspiCamCV.h"
#endif

//ZLib headers
#include <assert.h>
#include <zlib.h>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  namespace Video_Stream
  {
    using DUNE_NAMESPACES;
      
    struct Task: public DUNE::Tasks::Task
    {
      //!Variables
      #if raspicam_on == 1
      //RaspiCam config
      RASPIVID_CONFIG * config;
      //Capture struct - OpenCV/RaspiCAM
      RaspiCamCvCapture* capture;
      #else
      //Capture struct - OpenCV
      CvCapture* capture;
      #endif
      //Read time and data
      struct tm* local;
      //IplImage image_capture
      IplImage* img_IMC;
      //IplImage main
      IplImage* frame;
      //Buffer for Zlib Data
      IplImage* zlib_data;
      //Buffer for video frame
      CvVideoWriter *writer;
      //Define Font Letter OpenCV
      CvFont font;
      //Main frame width
      int frame_width;
      //Main frame height
      int frame_height;
      //width Inic
      int inic_width;
      //height Inic
      int inic_height;
      //Buffer text for frame result
      char text[80];
      //Buffer text for directory for log
      char local_dir[80];
      //Result of search local dir
      int str_dir;
      //User Name
      const char* user_name;
      //Global counter
      int cnt;
      //Flag - stat of video record
      bool flag_stat_video;
      //Flag - start record
      //!Variables Time
      //Hour
      int hour;
      //Minute
      int min;
      //Second
      int sec;
      //Day
      int day;
      //Month
      int mon;
      //Year
      int year;
      //Size of compress image
      unsigned long dsize;
      //Save info of compress API
      int result;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx)
      {
       /* param("Window Search Size", m_args.window_search_size)
        .defaultValue("55")
        .minimumValue("30")
        .maximumValue("155")
        .description("Window Search Size");
        
        param("Template Size", m_args.tpl_size)
        .defaultValue("25")
        .minimumValue("25")
        .maximumValue("150")
        .description("Template Size");
        
        param("Number of repetitions before the tpl refresh", m_args.rep_tpl)
        .defaultValue("6")
        .minimumValue("0")
        .maximumValue("12")
        .description("Number of repetitions before the tpl refresh");*/
        
        //bind<IMC::Tracking>(this);
      }
      
      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }
      
      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }
      
      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }
      
      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }
      
      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }
      
      //! Release resources.
      void
      onResourceRelease(void)
      {
      }
     
      //! Initialize Values
      void 
      InicValues(void)
      {
        flag_stat_video = 0;
        
        #if raspicam_on == 1
        config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        inic_width = 320;
        inic_height = 240;
        config->width = inic_width;
        config->height = inic_height;
        config->bitrate = 0; // zero: leave as default
        config->framerate = 12;
        config->monochrome = 0;
        #else
        inic_width = 640;
        inic_height = 480;
        #endif
      }
      
      /* Save Video Frame Result */
      void save_video(IplImage* image, bool parameter)
      {
        if (flag_stat_video == 0 && parameter == 1)
        {
          #ifdef linux
          sprintf(local_dir,"mkdir /home/$USER/%d_%d_%d_log_video -p",day,mon,year);
          str_dir = system(local_dir);
          user_name = getenv ("USER");
          sprintf(local_dir,"/home/%s/%d_%d_%d_log_video", user_name, day, mon, year);
          sprintf(text,"%s/%d_%d_%d___%d_%d_%d.avi",local_dir,hour,min,sec,day,mon,year);
          #endif
          
          #ifdef _WIN32
          str_dir = system("cd C:\ ");
          sprintf(local_dir,"mkdir %d_%d_%d_log_video",day,mon,year);
          str_dir = system(local_dir);
          sprintf(local_dir,"C:\%d_%d_%d_log_video",day,mon,year);
          sprintf(text,"%s\%d_%d_%d___%d_%d_%d.avi",local_dir,hour,min,sec,day,mon,year);
          #endif
          
          writer = cvCreateVideoWriter(text, CV_FOURCC('D','I','V','X'), 8, cvGetSize(image), 1);
          flag_stat_video = 1;
        }
        
        if (flag_stat_video == 1 && parameter == 1)
          cvWriteFrame(writer, image);      // add the frame to the file
        else if (flag_stat_video == 1 && parameter == 0)
        {
          cvReleaseVideoWriter( &writer );
          flag_stat_video = 0;
        }
      }
      
      /*Time acquisition */
      void
      time_acquisition(void)
      {
        time_t t;
        t = time(NULL);
        local = localtime(&t);
        
        hour = local -> tm_hour;
        min = local -> tm_min;
        sec = local -> tm_sec;
        day = local -> tm_mday;
        mon = local -> tm_mon + 1;
        year = local -> tm_year + 1900;
      }
      
      //! Main loop.
      void
      onMain(void)
      {
        //Initialize Values
        InicValues();
                
        #if raspicam_on == 1
        capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
        #else
        //capture = cvCaptureFromFile("rtsp://10.0.20.207:554/live/ch00_0"); //for airvision mini SENS-11
        //capture = cvCaptureFromCAM(0);//for laptop cam
        capture = cvCaptureFromFile("http://10.0.20.112/axis-cgi/mjpg/video.cgi?resolution=640x480&.mjpg"); //for axis cam
        //capture = cvCaptureFromFile("http://10.0.3.31:8080/video.wmv"); //for stream video
        #endif
        
        while ( capture  == 0 && !stopping())
        {
          inf("\n\tERROR OPEN CAM\n");
          #if raspicam_on == 1
          capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          capture = cvCaptureFromFile("http://10.0.20.112/axis-cgi/mjpg/video.cgi?resolution=640x480&.mjpg"); //for axis cam
          #endif
          cnt++;
          waitForMessages(1.0);
        }
        
        if ( capture )
        {
          //Capture Image
          #if raspicam_on == 1
          img = raspiCamCvQueryFrame(capture);
          cvReleaseImage( &frame );
          if (frame == 0 )
            frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
          cvResize(img, frame);
          #else
          frame = cvQueryFrame( capture );
          #endif
          
          //Size of Image capture
          frame_width = frame -> width;
          frame_height = frame -> height;
          inf("\n\tImage Size: %d x %d\n",frame_width, frame_height);
          
          //Font Opencv
          cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8);
        }
        
        cnt=0;
        while(cnt<300)
        {
          frame = cvQueryFrame( capture );
          cnt++;
        }
        time_acquisition();
        inf("\n\tStart... Hour: %d:%d:%d\n",hour,min,sec);
        while (!stopping())
        {
          #if raspicam_on == 1
          img = raspiCamCvQueryFrame(capture);
          cvReleaseImage( &frame );
          if (frame == 0 )
            frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
          cvResize(img, frame);
          #else
          frame = cvQueryFrame( capture );
          #endif
            
          if ( !capture )
          {
            inf("\n\tERROR GRAB IMAGE\n");
          }
            
          //Add information in frame result
          time_acquisition();
          sprintf(text,"Hour: %d:%d:%d",hour,min,sec);
          cvPutText(frame, text, cvPoint(10, 20), &font, cvScalar(150, 250, 150, 0));
          sprintf(text,"Data: %d/%d/%d",day,mon,year);
          cvPutText(frame, text, cvPoint(10, 42), &font, cvScalar(150, 250, 150, 0));
          text[0]='\0';
            
          //Send data by IMC
          /*msg.value_x = object_x + tpl_width/2;
          msg.value_y = object_y + tpl_height/2;
          dispatch(msg);*/
            
          //Save video

            save_video( frame, 1);
            //inf("\nSec: %d\n",sec);          
          
          /*
          if (zlib_data == NULL)
            zlib_data = cvCreateImage(cvGetSize(img_IMC),8,3);
            
          dsize = img_IMC->imageSize + (img_IMC->imageSize * 0.1f) + 20;
            
          result = compress2((unsigned char *)zlib_data->imageData, &dsize, (const unsigned char *)img_IMC->imageData, 
  back->imageSize,5);
            
          if(result != Z_OK)
            inf("\nCompress error occured!\n"); 
          */
          
          //cvReleaseImage(&zlib_data);
          //cvReleaseImage(&img_IMC);
          //cvShowImage("teste",frame);
          //cvWaitKey(8);
        }
        save_video( frame, 0);
        //cvDestroyWindow( "Live Video" );
        #if raspicam_on == 1
        raspiCamCvReleaseCapture( &capture );
        #else
        cvReleaseCapture(&capture);
        #endif
        
      }
    };
  }
}
DUNE_TASK
