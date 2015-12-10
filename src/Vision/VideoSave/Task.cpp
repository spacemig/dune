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

// DUNE headers.
#include <DUNE/Config.hpp>

//OpenCV headers
#include <opencv2/opencv.hpp>

//Enable(1) / disable(0) support for Raspicam
#if defined(DUNE_USING_RASPICAMCV)
#include "RaspiCamCV.h"
extern RaspiCamCvCapture* capture;
extern int flag_capture;
#endif

//ZLib headers
#include <assert.h>
#include <zlib.h>

//TCP-IP extra headers
#include <netdb.h>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  namespace VideoSave
  {
    using DUNE_NAMESPACES;
    struct Arguments
      {
        // - IpCam
        std::vector<std::string> ipcam;
      };
    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      //!Variables
      #if defined(DUNE_USING_RASPICAMCV)
      //RaspiCam config
      RASPIVID_CONFIG * config;
      //Capture struct - OpenCV/RaspiCAM
      //RaspiCamCvCapture* capture;
      //Buffer for video frame
      CvVideoWriter *writer;
      //Define Font Letter OpenCV
      CvFont font;
      //IplImage main
      IplImage* frame;
      #else
      //cv::Mat main frame
      cv::Mat frame;
      //Buffer for video frame
      cv::VideoWriter output_cap;
      //Capture struct - OpenCV
      cv::VideoCapture capture_mat;
      #endif
      //Read time and data
      struct tm* local;
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
      //IpCam Addresses
      const char* ipcam_addresses;
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
      //!Variables TCP-IP Socket
      int sockfd, portno, n;
      struct sockaddr_in serv_addr;
      struct hostent *server;
      //Buffer of tcp sender
      char buffer[30];
      // Flag state for send data
      bool ok_send;
      //Size of data received
      int tam_ok;
      //Counter for refresh sync
      int cnt_refresh_sync;
      //Start Time - Save func.
      double t1;
      //End Time - Save func.
      double t2;
      
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx)
      {
        param("IpCam", m_args.ipcam)
          //.defaultValue("localhost")
          .description("IpCam Addresses");
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
        for (unsigned int i = 0; i < m_args.ipcam.size(); ++i)
        {
          ipcam_addresses = m_args.ipcam[0].c_str();
        }
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
        
        #if defined(DUNE_USING_RASPICAMCV)
        /*config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        inic_width = 1280;
        inic_height = 720;
        config->width = inic_width;
        config->height = inic_height;
        config->bitrate = 0; // zero: leave as default
        config->framerate = 12;
        config->monochrome = 0;*/
        #else
        inic_width = 640;
        inic_height = 480;
        #endif
      }
      
      /* Save Video Frame Result */
      #if defined(DUNE_USING_RASPICAMCV)
      void save_video(IplImage* image, bool parameter)
      #else
      void save_video(cv::Mat image, bool parameter)
      #endif
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
          
          #if defined(DUNE_USING_RASPICAMCV)
          writer = cvCreateVideoWriter(text, CV_FOURCC('D','I','V','X'), 10, cvGetSize(image), 1);
          #else
          frame_height = image.rows;
          frame_width = image.cols;
          output_cap = cv::VideoWriter(text, CV_FOURCC('X', 'V', 'I', 'D'), 10, cvSize(frame_width, frame_height), 1);
          #endif
          flag_stat_video = 1;

        }
        
        if (flag_stat_video == 1 && parameter == 1)
        #if defined(DUNE_USING_RASPICAMCV)
          cvWriteFrame(writer, image);      // add the frame to the file
        #else
          output_cap.write(image);      // add the frame to the file
        #endif
        else if (flag_stat_video == 1 && parameter == 0)
        {
          #if defined(DUNE_USING_RASPICAMCV)
          cvReleaseVideoWriter( &writer );
          #else
          output_cap.release();
          #endif
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
      
      void
      clean_buffer(int value)
      {
        cnt=0;
        while(cnt<value)
        {
          #if defined(DUNE_USING_RASPICAMCV)
          //frame = cvQueryFrame( capture );
          #else
          capture_mat >> frame;
          #endif
          cnt++;
        }
        time_acquisition();
      }

      //! Main loop.
      void
      onMain(void)
      {        
        //Initialize Values
        InicValues();
        #if defined(DUNE_USING_RASPICAMCV)
        //capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
        //Font Opencv
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 2, 2, 0, 2, 8);
        #else
        capture_mat.open(ipcam_addresses);
        #endif
        
        #if defined(DUNE_USING_RASPICAMCV)
        while ( capture == 0 && !stopping())
        #else
        while ( !capture_mat.isOpened() && !stopping())
        #endif 
        {
          
          #if defined(DUNE_USING_RASPICAMCV)
          war(DTR("Waiting from task stream"));
          //capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          err(DTR("ERROR OPEN CAM"));
          capture_mat.open(ipcam_addresses);
          #endif
          cnt++;
          waitForMessages(1.0);
        }
        
        #if defined(DUNE_USING_RASPICAMCV)
        if ( capture )
        #else
        if ( capture_mat.isOpened() )
        #endif 
        {
          //Capture Image
          #if defined(DUNE_USING_RASPICAMCV)
          frame = raspiCamCvQueryFrame(capture);
          /*cvReleaseImage( &frame );
          if (frame == 0 )
            frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
          cvResize(img, frame);*/
          //Size of Image capture
          frame_width = frame -> width;
          frame_height = frame -> height;
          inf(DTR("Image Size: %d x %d"), frame_width, frame_height);
          #else
          //frame = cvQueryFrame( capture );
          capture_mat >> frame;
          frame_height = frame.rows;
          frame_width = frame.cols;
          inf(DTR("Image Size: %d x %d"), frame_width, frame_height);
          clean_buffer(50);
          #endif
        }
        
        cnt=1;
        time_acquisition();
        inf(DTR("Start Hour: %d:%d:%d"), hour, min, sec);

        while (!stopping())
        {
          t1=(double)cvGetTickCount();
          #if defined(DUNE_USING_RASPICAMCV)
          while(flag_capture == 2 && !stopping());
          flag_capture = 0;
          frame = raspiCamCvQueryFrame(capture);
          flag_capture = 1;
          #else
          capture_mat >> frame;
          #endif
          
          #if defined(DUNE_USING_RASPICAMCV)
          if ( !capture )
          {
            err(DTR("ERROR GRAB IMAGE"));
          }
          #else
          if ( !capture_mat.isOpened() )
          {
            err(DTR("ERROR GRAB IMAGE"));
          } 
          #endif
          
          //Add information in frame result
          time_acquisition();
          #if defined(DUNE_USING_RASPICAMCV)
          sprintf(text,"Hour: %d:%d:%d",hour,min,sec);
          cvPutText(frame, text, cvPoint(10, 20), &font, cvScalar(20, 90, 250, 0));
          sprintf(text,"Data: %d/%d/%d",day,mon,year);
          cvPutText(frame, text, cvPoint(10, 42), &font, cvScalar(20, 90, 250, 0));
          text[0]='\0';
          //Save video
          save_video(frame, 1);
          t2=(double)cvGetTickCount();
          while(((t2-t1)/(cvGetTickFrequency()*1000.))<(1000/10))
          {
            t2=(double)cvGetTickCount();
          }
          //inf("\ntime: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.)));
          #else
          sprintf(text,"Hour: %d:%d:%d",hour,min,sec);
          cv::putText(frame, text, cv::Point(10, 22), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 0),1.8, CV_AA);
          sprintf(text,"Data: %d/%d/%d",day,mon,year);
          cv::putText(frame, text, cv::Point(10, 42), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 0),1.8, CV_AA);
          text[0]='\0';
          //Save video
          save_video(frame, 1);
          t2=(double)cvGetTickCount();
          while(((t2-t1)/(cvGetTickFrequency()*1000.))<(1000/10))
          {
            t2=(double)cvGetTickCount();
          }
          //inf("\ntime: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.)));
          #endif
        }
        save_video( frame, 0);
        //cvDestroyWindow( "Live Video" );
        #if defined(DUNE_USING_RASPICAMCV)
        //raspiCamCvReleaseCapture( &capture );
        #else
        capture_mat.release();
        #endif
      }
    };
  }
}
DUNE_TASK
