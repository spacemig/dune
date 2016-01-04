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
extern RaspiCamCvCapture* m_capture;
extern int m_flag_capture;
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
      // - port number
      unsigned printimglayer;
      // - IpCam
      std::vector<std::string> ipcam;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      //!Variables
      #if defined(DUNE_USING_RASPICAMCV)
      //RaspiCam config
      RASPIVID_CONFIG * m_config;
      //Capture struct - OpenCV/RaspiCAM
      //RaspiCamCvCapture* capture;
      //Buffer for video frame
      CvVideoWriter *m_writer;
      //Define Font Letter OpenCV
      CvFont m_font;
      //IplImage main
      IplImage* m_frame;
      #else
      //cv::Mat main frame
      cv::Mat m_frame;
      //Buffer for video frame
      cv::VideoWriter m_output_cap;
      //Capture struct - OpenCV
      cv::VideoCapture m_capture_mat;
      #endif
      //Read time and data
      struct tm* m_local;
      //Main frame width
      int m_frame_width;
      //Main frame height
      int m_frame_height;
      //width Inic
      int m_inic_width;
      //height Inic
      int m_inic_height;
      //Buffer text for frame result
      char m_text[80];
      //Buffer text for directory for log
      char m_local_dir[80];
      //Result of search local dir
      int m_str_dir;
      //User Name
      const char* m_user_name;
      //IpCam Addresses
      const char* m_ipcam_addresses;
      //Global counter
      int m_cnt;
      //Flag - stat of video record
      bool m_flag_stat_video;
      //Flag - start record
      //!Variables Time
      //Hour
      int m_hour;
      //Minute
      int m_min;
      //Second
      int m_sec;
      //Day
      int m_day;
      //Month
      int m_mon;
      //Year
      int m_year;
      //Size of compress image
      unsigned long m_dsize;
      //Save info of compress API
      int m_result;
      //!Variables TCP-IP Socket
      int m_sockfd, m_portno, m_n;
      struct sockaddr_in m_serv_addr;
      struct hostent *m_server;
      //Buffer of tcp sender
      char m_buffer[30];
      // Flag state for send data
      bool m_ok_send;
      //Size of data received
      int m_tam_ok;
      //Counter for refresh sync
      int m_cnt_refresh_sync;
      //Start Time - Save func.
      double m_t1;
      //End Time - Save func.
      double m_t2;
      
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx)
      {
        param("IpCam", m_args.ipcam)
          //.defaultValue("localhost")
          .description("IpCam Addresses");

        param("PrintImgLayer", m_args.printimglayer)
          //.defaultValue("localhost")
          .description("Disable/Enable image layer print");
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
          m_ipcam_addresses = m_args.ipcam[0].c_str();
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
      inicValues(void)
      {
        m_flag_stat_video = 0;
        
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
        m_inic_width = 640;
        m_inic_height = 480;
        #endif
      }
      
      /* Save Video Frame Result */
      #if defined(DUNE_USING_RASPICAMCV)
      void saveVideo(IplImage* image, bool parameter)
      #else
      void saveVideo(cv::Mat image, bool parameter)
      #endif
      {
        if (m_flag_stat_video == 0 && parameter == 1)
        {
          #ifdef linux
          sprintf(m_local_dir,"mkdir /home/$USER/%d_%d_%d_log_video -p",m_day,m_mon,m_year);
          m_str_dir = system(m_local_dir);
          m_user_name = getenv ("USER");
          sprintf(m_local_dir,"/home/%s/%d_%d_%d_log_video", m_user_name, m_day, m_mon, m_year);
          sprintf(m_text,"%s/%d_%d_%d___%d_%d_%d.avi",m_local_dir, m_hour, m_min, m_sec, m_day, m_mon, m_year);
          #endif
          
          #ifdef _WIN32
          m_str_dir = system("cd C:\ ");
          sprintf(m_local_dir,"mkdir %d_%d_%d_log_video",m_day, m_mon, m_year);
          m_str_dir = system(m_local_dir);
          sprintf(m_local_dir,"C:\%d_%d_%d_log_video",m_day, m_mon, m_year);
          sprintf(m_text,"%s\%d_%d_%d___%d_%d_%d.avi",m_local_dir, m_hour, m_min, m_sec, m_day, m_mon, m_year);
          #endif
          
          #if defined(DUNE_USING_RASPICAMCV)
          m_writer = cvCreateVideoWriter(m_text, CV_FOURCC('D','I','V','X'), 10, cvGetSize(image), 1);
          #else
          m_frame_height = image.rows;
          m_frame_width = image.cols;
          m_output_cap = cv::VideoWriter(m_text, CV_FOURCC('X', 'V', 'I', 'D'), 10, cvSize(m_frame_width, m_frame_height), 1);
          #endif
          m_flag_stat_video = 1;

        }
        
        if (m_flag_stat_video == 1 && parameter == 1)
        #if defined(DUNE_USING_RASPICAMCV)
          cvWriteFrame(m_writer, image);      // add the frame to the file
        #else
          m_output_cap.write(image);      // add the frame to the file
        #endif
        else if (m_flag_stat_video == 1 && parameter == 0)
        {
          #if defined(DUNE_USING_RASPICAMCV)
          cvReleaseVideoWriter( &m_writer );
          #else
          m_output_cap.release();
          #endif
          m_flag_stat_video = 0;
        }
      }
      
      /*Time acquisition */
      void
      timeAcquisition(void)
      {
        time_t t;
        t = time(NULL);
        m_local = localtime(&t);
        
        m_hour = m_local -> tm_hour;
        m_min = m_local -> tm_min;
        m_sec = m_local -> tm_sec;
        m_day = m_local -> tm_mday;
        m_mon = m_local -> tm_mon + 1;
        m_year = m_local -> tm_year + 1900;
      }
      
      void
      cleanBuffer(int value)
      {
        m_cnt=0;
        while(m_cnt < value)
        {
          #if defined(DUNE_USING_RASPICAMCV)
          //frame = cvQueryFrame( capture );
          #else
          m_capture_mat >> m_frame;
          #endif
          m_cnt++;
        }
        timeAcquisition();
      }

      //! Main loop.
      void
      onMain(void)
      {        
        //Initialize Values
        inicValues();
        #if defined(DUNE_USING_RASPICAMCV)
        //capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
        //Font Opencv
        cvInitFont(&m_font, CV_FONT_HERSHEY_PLAIN, 2, 2, 0, 2, 8);
        #else
        m_capture_mat.open(m_ipcam_addresses);
        #endif
        
        #if defined(DUNE_USING_RASPICAMCV)
        while ( m_capture == 0 && !stopping())
        #else
        while ( !m_capture_mat.isOpened() && !stopping())
        #endif 
        {
          #if defined(DUNE_USING_RASPICAMCV)
          war(DTR("Waiting from task stream"));
          //capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          err(DTR("ERROR OPEN CAM"));
          m_capture_mat.open(m_ipcam_addresses);
          #endif
          m_cnt++;
          waitForMessages(1.0);
        }
        
        #if defined(DUNE_USING_RASPICAMCV)
        if ( m_capture )
        #else
        if ( m_capture_mat.isOpened() )
        #endif 
        {
          //Capture Image
          #if defined(DUNE_USING_RASPICAMCV)
          m_frame = raspiCamCvQueryFrame(m_capture);
          /*cvReleaseImage( &frame );
          if (frame == 0 )
            frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
          cvResize(img, frame);*/
          //Size of Image capture
          m_frame_width = m_frame -> width;
          m_frame_height = m_frame -> height;
          inf(DTR("Image Size: %d x %d"), m_frame_width, m_frame_height);
          #else
          //frame = cvQueryFrame( capture );
          m_capture_mat >> m_frame;
          m_frame_height = m_frame.rows;
          m_frame_width = m_frame.cols;
          inf(DTR("Image Size: %d x %d"), m_frame_width, m_frame_height);
          cleanBuffer(50);
          #endif
        }
        
        m_cnt = 1;
        timeAcquisition();
        inf(DTR("Start Hour: %d:%d:%d"), m_hour, m_min, m_sec);

        while (!stopping())
        {
          m_t1=(double)cvGetTickCount();
          #if defined(DUNE_USING_RASPICAMCV)
          while(m_flag_capture == 2 && !stopping());
          m_flag_capture = 0;
          m_frame = raspiCamCvQueryFrame(m_capture);
          m_flag_capture = 1;
          #else
          m_capture_mat >> m_frame;
          #endif
          
          #if defined(DUNE_USING_RASPICAMCV)
          if ( !m_capture )
          {
            err(DTR("ERROR GRAB IMAGE"));
          }
          #else
          if ( !m_capture_mat.isOpened() )
          {
            err(DTR("ERROR GRAB IMAGE"));
          } 
          #endif
          
          //Add information in frame result
          timeAcquisition();
          #if defined(DUNE_USING_RASPICAMCV)
          if(m_args.printimglayer == 1)
          {
            sprintf(m_text,"Hour: %d:%d:%d", m_hour, m_min, m_sec);
            cvPutText(m_frame, m_text, cvPoint(10, 20), &m_font, cvScalar(20, 90, 250, 0));
            sprintf(m_text,"Data: %d/%d/%d", m_day, m_mon, m_year);
            cvPutText(m_frame, m_text, cvPoint(10, 42), &m_font, cvScalar(20, 90, 250, 0));
            m_text[0]='\0'; 
          }
          //Save video
          saveVideo(m_frame, 1);
          m_t2=(double)cvGetTickCount();
          while(((m_t2-m_t1)/(cvGetTickFrequency()*1000.))<(1000/10))
          {
            m_t2=(double)cvGetTickCount();
          }
          //inf("\ntime: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.)));
          #else
          if(m_args.printimglayer == 1)
          { 
            sprintf(m_text,"Hour: %d:%d:%d", m_hour, m_min, m_sec);
            cv::putText(m_frame, m_text, cv::Point(10, 22), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 0),1.8, CV_AA);
            sprintf(m_text,"Data: %d/%d/%d", m_day, m_mon, m_year);
            cv::putText(m_frame, m_text, cv::Point(10, 42), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255, 0),1.8, CV_AA);
            m_text[0]='\0';
          }
          //Save video
          saveVideo(m_frame, 1);
          m_t2=(double)cvGetTickCount();
          while(((m_t2-m_t1)/(cvGetTickFrequency()*1000.))<(1000/10))
          {
            m_t2=(double)cvGetTickCount();
          }
          //inf("\ntime: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.)));
          #endif
        }
        saveVideo( m_frame, 0);
        //cvDestroyWindow( "Live Video" );
        #if defined(DUNE_USING_RASPICAMCV)
        //raspiCamCvReleaseCapture( &capture );
        #else
        m_capture_mat.release();
        #endif
      }
    };
  }
}
DUNE_TASK
