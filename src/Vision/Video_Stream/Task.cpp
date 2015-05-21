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
#include <pthread.h>

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
  namespace Video_Stream
  {
    using DUNE_NAMESPACES;
    struct Arguments
      {
        // - host name
        std::vector<std::string> host;
      };
    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      //!Variables
      #if raspicam_on == 1
      //RaspiCam config
      RASPIVID_CONFIG * config;
      //Capture struct - OpenCV/RaspiCAM
      RaspiCamCvCapture* capture;
      //Buffer for video frame
      CvVideoWriter *writer;
      #else
      //Capture struct - OpenCV
      CvCapture* capture;
      #endif
      //Read time and data
      struct tm* local;
      //IplImage image_capture
      IplImage* img;
      //IplImage main
      IplImage* frame;
      //Buffer for Zlib Data
      IplImage* zlib_data;
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
      //Host Name
      const char* host_name;
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
      //Latitude
      float lat;
      //Longitude
      float lon;
      
      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx)
      {
        param("Host", m_args.host)
          //.defaultValue("localhost")
          .description("Name of the client in network");
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
        
        bind<IMC::EstimatedState>(this);
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
        for (unsigned int i = 0; i < m_args.host.size(); ++i)
        {
          host_name = m_args.host[0].c_str();
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
      
      void
      consume(const IMC::EstimatedState* msg)
      {
        //inf("Source (DUNE instance) ID is: %d", msg->getSource());
        //inf("Source entity (Task instance) is: %s", resolveEntity(msg->getSourceEntity()).c_str());
        lat = msg->lat;
        lon = msg->lon;
        //inf("Lat = %f   Lon = %f", lat, lon);
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
      
      /* Inic TCP connections */
      void 
      InicTCP(bool state)
      {
        if (!state)
        {
          portno = 2424;
          sockfd = socket(AF_INET, SOCK_STREAM, 0);
          if (sockfd < 0)
          {
            inf("ERROR opening socket");
            exit(0);
          }
          server = gethostbyname(host_name);
          if (server == NULL)
          {
            inf("ERROR, no such host");
            exit(0);
          }
          inf("Wainting connection TCP-IP...");
          bzero((char *) &serv_addr, sizeof(serv_addr));
          serv_addr.sin_family = AF_INET;
          bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
          serv_addr.sin_port = htons(portno);
          while(connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0 && !stopping())
          {
            waitForMessages(0);
          }
          inf("Connection TCP-IP ON");
        }
        else
        {
          //Send info size image over tcp
          inf("Sending Data image info...");
          sprintf(buffer,"%d\n",frame_width);
          n = send(sockfd, buffer, strlen(buffer), 0);
          if (n < 0)
          {
            inf("ERROR writing to socket: Image Size Width");
            exit(0);
          }
          sprintf(buffer,"%d\n",frame_height);
          n = send(sockfd, buffer, strlen(buffer), 0);
          if (n < 0)
          {
            inf("ERROR writing to socket: Image Size Height");
            exit(0);
          }
          inf("Sending Data image info Done (%d x %d) ...",frame_width, frame_height);
        }
      }
      
      #if raspicam_on == 1
      /* Save Video Frame Result */
      void save_video(IplImage* image, bool parameter)
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
          
          writer = cvCreateVideoWriter(text, CV_FOURCC('D','I','V','X'), 10, cvGetSize(image), 1);
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
      #endif

      void
      clean_buffer(int value)
      {
        cnt=0;
        while(cnt<value)
        {
          cvGrabFrame(capture);
          //frame = cvQueryFrame( capture );
          cnt++;
        }
      }

      void
      inic_Capture(void)
      {
        #if raspicam_on == 1
        capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
        #else
        //cvSetCaptureProperty( capture, 5, 8);
        //capture = cvCaptureFromFile("rtsp://10.0.20.207:554/live/ch00_0"); //for airvision mini SENS-11
        //capture = cvCaptureFromCAM(0);//for laptop cam
        //capture = cvCaptureFromFile("http://10.0.20.102/axis-cgi/mjpg/video.cgi?resolution=320x240&.mjpg"); //for axis cam
        //capture = cvCaptureFromFile("http://10.0.3.31:8080/video.wmv"); //for stream video640x480
        capture = cvCaptureFromFile("rtsp://10.0.20.102:554/axis-media/media.amp?streamprofile=Mobile");
        #endif
        
        while ( capture  == 0 && !stopping())
        {
          inf("ERROR OPEN CAM\n");
          #if raspicam_on == 1
          capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          //cvSetCaptureProperty( capture, 5, 8);
          //capture = cvCaptureFromFile("http://10.0.20.102/axis-cgi/mjpg/video.cgi?resolution=320x240&.mjpg"); //for axis cam
          capture = cvCaptureFromFile("rtsp://10.0.20.102:554/axis-media/media.amp?streamprofile=Mobile");
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
          //cvSetCaptureProperty( capture, 5, 8);
          clean_buffer(50);
          frame = cvQueryFrame( capture );
          #endif
          
          //Size of Image capture
          frame_width = frame -> width;
          frame_height = frame -> height;
          inf("Image Size: %d x %d \t TASK: STREAM",frame_width, frame_height);
        }
      }
      //! Main loop.
      void
      onMain(void)
      {
        //inf("\n >>>>  Nome : %s  <<<< \n\n",m_args.host[0].c_str());
        //IMC::EstimatedState msg;
        //Initialize Values 
        InicValues();
        InicTCP(0);
        
        inic_Capture();
                
        InicTCP(1);
        
        while (!stopping())
        {
          while(n >= 0 && !stopping())
          {
            waitForMessages(0);
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
              inf("ERROR GRAB IMAGE");
            }            
       
            if (zlib_data == NULL)
              zlib_data = cvCreateImage(cvGetSize(frame),8,3);
            
            dsize = frame->imageSize + (frame->imageSize * 0.1f) + 20;
            
            result = compress2((unsigned char *)zlib_data->imageData, &dsize, (const unsigned char 
            *)frame->imageData, frame->imageSize, 5);
       
            if(result != Z_OK)
              inf("Compress error occured!");
            
            //send data size
            sprintf(buffer,"%ld\n",dsize);
            n = send(sockfd, buffer, strlen(buffer), 0);
            if (n < 0)
              inf("ERROR writing to socket: Send Data Size Image");
            
            ok_send = 0;
            tam_ok=0;
            cnt_refresh_sync=0;
            while(ok_send == 0)
            {
              ///recv data for sync and debug
              n = recv(sockfd, buffer, 20, 0);
              tam_ok = strlen(buffer);
              if (n <= 0)
                inf("ERROR reading of socket");
              
              if(tam_ok == 1)
              {    
                ok_send = atoi(buffer);
              }
              
              cnt_refresh_sync++;
              if (cnt_refresh_sync > 6)
              {
                ok_send = 1;
                //inf("\nRefresh Sync...\n");
              }
            }
            
            //send data image
            n = send(sockfd, zlib_data->imageData, dsize, 0);
            //        printf("\nSend %d OK %d",n,dsize);
            if (n < 0)
              inf("ERROR writing to socket");
            
            cvReleaseImage(&zlib_data);
            #if raspicam_on == 1
            save_video(frame,1);
            #endif
            //cvWaitKey(20);
          }
          if(!stopping())
          {
            close(sockfd);
            #if raspicam_on == 1
            raspiCamCvReleaseCapture( &capture );
            #else
            cvReleaseCapture(&capture);
            #endif
            inf("Restarting connection TCP-IP...");
            InicTCP(0);
            InicTCP(1);
            inic_Capture();
          }
        }
        #if raspicam_on == 1
        raspiCamCvReleaseCapture( &capture );
        save_video(frame,0);
        #else
        cvReleaseCapture(&capture);
        #endif
        close(sockfd);
      }
    };
  }
}
DUNE_TASK
