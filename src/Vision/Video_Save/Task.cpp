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

//TCP-IP extra headers
#include <netdb.h>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  namespace Video_Save
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
        /* param("Window Search Size", m_args.window_search_size)
         *        .defaultValue("55")
         *        .minimumValue("30")
         *        .maximumValue("155")
         *        .description("Window Search Size");
         *        
         *        param("Template Size", m_args.tpl_size)
         *        .defaultValue("25")
         *        .minimumValue("25")
         *        .maximumValue("150")
         *        .description("Template Size");
         *        
         *        param("Number of repetitions before the tpl refresh", m_args.rep_tpl)
         *        .defaultValue("6")
         *        .minimumValue("0")
         *        .maximumValue("12")
         *        .description("Number of repetitions before the tpl refresh");*/
        
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
        inic_width = 1280;
        inic_height = 720;
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
      #if raspicam_on == 1
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
          
          #if raspicam_on == 1
          writer = cvCreateVideoWriter(text, CV_FOURCC('D','I','V','X'), 10, cvGetSize(image), 1);
          #else
          frame_height = image.rows;
          frame_width = image.cols;
          output_cap = cv::VideoWriter(text, CV_FOURCC('X', 'V', 'I', 'D'), 10, cvSize(frame_width, frame_height), 1);
          #endif
          flag_stat_video = 1;

        }
        
        if (flag_stat_video == 1 && parameter == 1)
        #if raspicam_on == 1
          cvWriteFrame(writer, image);      // add the frame to the file
        #else
          output_cap.write(image);      // add the frame to the file
        #endif
        else if (flag_stat_video == 1 && parameter == 0)
        {
          #if raspicam_on == 1
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
          capture_mat >> frame;
          //frame = cvQueryFrame( capture );
          cnt++;
        }
        time_acquisition();
        inf("Start... Hour: %d:%d:%d \t TASK: STREAM",hour,min,sec);
      }

      //! Main loop.
      void
      onMain(void)
      {        
        //IMC::CompressedImage msg;
        //Initialize Values
        InicValues();
        #if raspicam_on == 1
        capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
        //Font Opencv
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 8);
        #else
        //cvSetCaptureProperty( capture, 5, 16);
        //capture = cvCaptureFromFile("rtsp://10.0.20.207:554/live/ch00_0"); //for airvision mini SENS-11
        //capture = cvCaptureFromCAM(0);//for laptop cam
        //capture = cvCaptureFromFile("rtsp://10.0.20.112:554/axis-media/media.amp"); //for axis cam
        //capture = cvCaptureFromFile("http://10.0.20.112/axis-cgi/mjpg/video.cgi?resolution=1280x720&.mjpg"); //for axis cam
        capture_mat.open("rtsp://10.0.20.102:554/axis-media/media.amp?streamprofile=Bandwidth");
        #endif
        
        #if raspicam_on == 1
        while ( capture == 0 && !stopping())
        #else
        while ( !capture_mat.isOpened() && !stopping())
        #endif 
        {
          inf("\n\tERROR OPEN CAM\n");
          #if raspicam_on == 1
          capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          //cvSetCaptureProperty( capture, 5, 16);
          //capture = cvCaptureFromFile("rtsp://10.0.20.112:554/axis-media/media.amp"); //for axis cam
          //capture = cvCaptureFromFile("http://10.0.20.112/axis-cgi/mjpg/video.cgi?resolution=1280x720&.mjpg"); //for axis cam
          capture_mat.open("rtsp://10.0.20.102:554/axis-media/media.amp?streamprofile=Bandwidth");
          #endif
          cnt++;
          waitForMessages(1.0);
        }
        
        #if raspicam_on == 1
        if ( capture )
        #else
        if ( capture_mat.isOpened() )
        #endif 
        {
          //Capture Image
          #if raspicam_on == 1
          frame = raspiCamCvQueryFrame(capture);
       /*   cvReleaseImage( &frame );
          if (frame == 0 )
            frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
          cvResize(img, frame);*/
          //Size of Image capture
          frame_width = frame -> width;
          frame_height = frame -> height;
          inf("Image Size: %d x %d\t TASK: SAVE",frame_width, frame_height);
          #else
          //frame = cvQueryFrame( capture );
          capture_mat >> frame;
          frame_height = frame.rows;
          frame_width = frame.cols;
          inf("Image Size: %d x %d\t TASK: SAVE",frame_width, frame_height);
          clean_buffer(50);
          #endif
        }
        
        cnt=1;
        time_acquisition();
        inf("Start... Hour: %d:%d:%d \t TASK: SAVE",hour,min,sec);

        while (!stopping())
        {
          t1=(double)cvGetTickCount();
          #if raspicam_on == 1
          frame = raspiCamCvQueryFrame(capture);
          #else
          capture_mat >> frame;
          #endif
          
          #if raspicam_on == 1
          if ( !capture )
          {
            inf("ERROR GRAB IMAGE");
          }
          #else
          if ( !capture_mat.isOpened() )
          {
            inf("ERROR GRAB IMAGE");
          } 
          #endif
          

          //Add information in frame result
          time_acquisition();
          #if raspicam_on == 1
          sprintf(text,"Hour: %d:%d:%d",hour,min,sec);
          cvPutText(frame, text, cvPoint(10, 20), &font, cvScalar(20, 90, 250, 0));
          sprintf(text,"Data: %d/%d/%d",day,mon,year);
          cvPutText(frame, text, cvPoint(10, 42), &font, cvScalar(20, 90, 250, 0));
          text[0]='\0';
          //Save video
          save_video(frame, 1);
          #else
          sprintf(text,"Hour: %d:%d:%d",hour,min,sec);
          cv::putText(frame, text, cv::Point(10, 22), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(250, 90, 20, 0),1.8, CV_AA);
          sprintf(text,"Data: %d/%d/%d",day,mon,year);
          cv::putText(frame, text, cv::Point(10, 42), CV_FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(250, 90, 20, 0),1.8, CV_AA);
          text[0]='\0';
          //Save video
          save_video(frame, 1);
          t2=(double)cvGetTickCount();
          while(((t2-t1)/(cvGetTickFrequency()*1000.))<(1000/11))
          {
            t2=(double)cvGetTickCount();
          }
          //inf("time: %gms  fps: %.2g\n",(t2-t1)/(cvGetTickFrequency()*1000.), 1000./((t2-t1)/(cvGetTickFrequency()*1000.)));
        
          #endif
        }
        save_video( frame, 0);
        //cvDestroyWindow( "Live Video" );
        #if raspicam_on == 1
        raspiCamCvReleaseCapture( &capture );
        #else
        capture_mat.release();
        #endif
      }
    };
  }
}
DUNE_TASK