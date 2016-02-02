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

// DUNE headers.
#include <DUNE/Config.hpp>

//OpenCV headers
#include <opencv2/opencv.hpp>

//Enable(1) / disable(0) support for Raspicam
#if defined(DUNE_USING_RASPICAMCV)
//RaspiCAM headers
#include "RaspiCamCV.h"
RaspiCamCvCapture* m_capture;
int m_flag_capture = 0;
#endif

//ZLib headers
#include <assert.h>
#include <zlib.h>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  namespace VideoStream
  {
    using DUNE_NAMESPACES;

    // Client data.
    struct Client
    {
      int size;       // size of data received.
      char data[16];  // Data.
    };

    struct Arguments
    {
      // - port number
      unsigned portno;
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
      #else
      //Capture struct - OpenCV
      CvCapture* m_capture;
      #endif
      //Read time and data
      struct tm* m_local;
      //IplImage image_capture
      IplImage* m_img;
      //IplImage main
      IplImage* m_frame;
      //Buffer for Zlib Data
      IplImage* m_zlib_data;
      //Define Font Letter OpenCV
      CvFont m_font;
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
      //Host Name
      const char* m_host_name;
      //IpCam Addresses
      const char* m_ipcam_addresses;
      //User Name
      const char* m_user_name;
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
      //Buffer of tcp sender
      char m_buffer[64];
      // Flag state for send data
      bool m_ok_send;
      //Size of data received
      int m_tam_ok;
      //Counter for refresh sync
      int m_cnt_refresh_sync;
      //Latitude
      double m_lat;
      //Longitude
      double m_lon;
      //Height
      double m_heig;
      //Start Time - Save func.
      double m_t1;
      //End Time - Save func.
      double m_t2;
      // Socket handle.
      TCPSocket* m_sock;
      // I/O Multiplexer.
      Poll m_poll;
      // Clients.
      std::list<TCPSocket*> m_clients;
      //State of TCPsocket
      int m_stateTcpIni;
      
      Client m_client;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
      DUNE::Tasks::Task(name, ctx),
      m_sock(NULL)
      {
        param("Port", m_args.portno)
          //.defaultValue("localhost")
          .description("Port to use in TCP-IP");

        param("IpCam", m_args.ipcam)
          //.defaultValue("localhost")
          .description("IpCam Addresses");
        
        
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
        for (unsigned int i = 0; i < m_args.ipcam.size(); ++i)
        {
          m_ipcam_addresses = m_args.ipcam[0].c_str();
        }
      }
      
      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_sock = new TCPSocket;
      }
      
      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        m_sock->bind(m_args.portno);
        m_sock->listen(5);
        m_poll.add(*m_sock);
      }
      
      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (m_sock != NULL)
        {
          m_poll.remove(*m_sock);
          delete m_sock;
          m_sock = NULL;
        }

        std::list<TCPSocket*>::iterator itr = m_clients.begin();
        for (; itr != m_clients.end(); ++itr)
        {
          m_poll.remove(*(*itr));
          delete *itr;
        }

        m_clients.clear();
      }
      
      void
      dispatchToClients(const char* bfr, unsigned bfr_len)
      {
        std::list<TCPSocket*>::iterator itr = m_clients.begin();
        while (itr != m_clients.end())
        {
          try
          {
            (*itr)->write(bfr, bfr_len);
            ++itr;
          }
          catch (Network::ConnectionClosed& e)
          {
              err(DTR("Connection closed"));
              m_client.size = -1;
              (void)e;
              m_poll.remove(*(*itr));
              delete *itr;
              itr = m_clients.erase(itr);
              continue;
          }
          catch (std::runtime_error& e)
          {
            m_client.size = -1;
            err("%s", e.what());
            m_poll.remove(*(*itr));
            delete *itr;
            itr = m_clients.erase(itr);
          }
        }
      }

      void
      checkMainSocket(void)
      {
        bool stateCom = false;
        while(!stateCom)
        {
          if (m_poll.wasTriggered(*m_sock))
          {
            inf(DTR("accepting connection request"));
            try
            {
              TCPSocket* nc = m_sock->accept();
              m_clients.push_back(nc);
              m_poll.add(*nc);
              stateCom = true;
            }
            catch (std::runtime_error& e)
            {
              err("%s", e.what());
            }
          }
        }
      }

      Client
      checkClientSockets(void)
      {
        Client  bfr;
        memset(bfr.data, '\0', sizeof(bfr.data));

        std::list<TCPSocket*>::iterator itr = m_clients.begin();
        while (itr != m_clients.end())
        {
          if (!m_poll.poll(1.0))
          {
            err("time out while waiting for reply");
            bfr.size = -1;
          }
          if (m_poll.wasTriggered(*(*itr)))
          {
            try
            {
              bfr.size = (*itr)->read(bfr.data, sizeof(bfr.data));
              //inf("Valor recebido: %s  |  size: %d", bfr.data, bfr.size);
            }
            catch (Network::ConnectionClosed& e)
            {
              err(DTR("Connection closed"));
              bfr.size = -1;
              (void)e;
              m_poll.remove(*(*itr));
              delete *itr;
              itr = m_clients.erase(itr);
              continue;
            }
            catch (std::runtime_error& e)
            {
              err("%s", e.what());
              bfr.size = -1;
            }
          }

          ++itr;
        }
        return bfr;
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        //! update the position of vehicle
        //LAT and LON rad
        double latRad = msg->lat;
        double lonRad = msg->lon;
        m_heig = (msg->height) - (msg->z);
        //LAT and LON rad
        //LAT and LON deg
        double latDeg = DUNE::Math::Angles::degrees(latRad);
        double lonDeg = DUNE::Math::Angles::degrees(lonRad);
        //Offset (m)
        double offsetN = msg->x;
        double offsetE = msg->y;
        //Lat and Lon final
        m_lat = latDeg + (180/M_PI)*(offsetE/6378137);
        m_lon = lonDeg + (180/M_PI)*(offsetN/6378137)/cos(latDeg);
        //inf("Lat = %f   Lon = %f", lat, lon);
      }
      //! Initialize Values
      void 
      inicValues(void)
      {
        m_stateTcpIni = 0;
        m_client.size = 0;
        m_flag_stat_video = 0;
        
        #if defined(DUNE_USING_RASPICAMCV)
        m_config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        m_inic_width = 160;
        m_inic_height = 120;
        m_config->width = 640;
        m_config->height = 480;
        m_config->bitrate = 0; // zero: leave as default
        //config->framerate = 20;
        m_config->monochrome = 0;
        #else
        m_inic_width = 160;
        m_inic_height = 120;
        #endif
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
      
      /* Inic TCP connections */
      void 
      inicTCP(bool state)
      {
        if (!state)
        {
          inf(DTR("Waiting from client"));
          while(m_stateTcpIni == 0 && !stopping())
          {
            if (m_poll.poll(0.01) && m_stateTcpIni == 0)
            {
              checkMainSocket();
              m_stateTcpIni = 1;
            }
          }
          inf(DTR("Connection TCP-IP ON"));
        }
        else
        {
          //Send info size image over tcp
          inf(DTR("Sending Data image info"));
          sprintf(m_buffer,"%d\n", m_frame_width);
          dispatchToClients(m_buffer, strlen(m_buffer));
          
          sprintf(m_buffer,"%d\n", m_frame_height);
          dispatchToClients(m_buffer, strlen(m_buffer));

          inf(DTR("Image Size: %dx%d"), m_frame_width, m_frame_height);
          m_stateTcpIni = 0;
        }
      }
      
      #if defined(DUNE_USING_RASPICAMCV)
      /* Save Video Frame Result */
      void saveVideo(IplImage* image, bool parameter)
      {
        if (m_flag_stat_video == 0 && parameter == 1)
        {
          timeAcquisition();
          #ifdef linux
          sprintf(m_local_dir,"mkdir /home/$USER/%d_%d_%d_log_video -p", m_day, m_mon, m_year);
          m_str_dir = system(m_local_dir);
          m_user_name = getenv ("USER");
          sprintf(m_local_dir,"/home/%s/%d_%d_%d_log_video", m_user_name, m_day, m_mon, m_year);
          sprintf(m_text,"%s/%d_%d_%d___%d_%d_%d.avi",m_local_dir, m_hour, m_min, m_sec, m_day, m_mon, m_year);
          #endif
          
          #ifdef _WIN32
          m_str_dir = system("cd C:\ ");
          sprintf(m_local_dir,"mkdir %d_%d_%d_log_video", m_day, m_mon, m_year);
          m_str_dir = system(m_local_dir);
          sprintf(m_local_dir,"C:\%d_%d_%d_log_video", m_day, m_mon, m_year);
          sprintf(m_text,"%s\%d_%d_%d___%d_%d_%d.avi",m_local_dir, m_hour, m_min, m_sec, m_day, m_mon, m_year);
          #endif
          
          m_writer = cvCreateVideoWriter(m_text, CV_FOURCC('D','I','V','X'), 10, cvGetSize(image), 1);
          m_flag_stat_video = 1;
        }
        
        if (m_flag_stat_video == 1 && parameter == 1)
          cvWriteFrame(m_writer, image);      // add the frame to the file
        else if (m_flag_stat_video == 1 && parameter == 0)
        {
          cvReleaseVideoWriter( &m_writer );
          m_flag_stat_video = 0;
        }
      }
      #endif

      void
      cleanBuffer(int value)
      {
        m_cnt=0;
        while(m_cnt<value)
        {
          #if defined(DUNE_USING_RASPICAMCV)
          //frame = cvQueryFrame( capture );
          #else
          cvGrabFrame(m_capture);
          #endif
          m_cnt++;
        }
      }

      void
      inicCapture(void)
      {
        #if defined(DUNE_USING_RASPICAMCV)
        m_capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, m_config);
        #else
        m_capture = cvCaptureFromFile(m_ipcam_addresses);
        #endif
        
        while ( m_capture  == 0 && !stopping())
        {
          err(DTR("ERROR OPEN CAM\n"));
          #if defined(DUNE_USING_RASPICAMCV)
          m_capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, m_config);
          #else
          m_capture = cvCaptureFromFile(m_ipcam_addresses);
          #endif
          m_cnt++;
          waitForMessages(1.0);
        }
        
        if ( m_capture )
        {
          //Capture Image
          #if defined(DUNE_USING_RASPICAMCV)
          m_img = raspiCamCvQueryFrame(m_capture);
          cvReleaseImage( &m_frame );
          if (m_frame == 0 )
            m_frame = cvCreateImage ( cvSize(m_inic_width, m_inic_height), m_img -> depth, m_img -> nChannels);
          cvResize(m_img, m_frame);
          #else
          //cvSetCaptureProperty( capture, 5, 8);
          cleanBuffer(50);
          m_frame = cvQueryFrame( m_capture );
          #endif
          
          //Size of Image capture
          m_frame_width = m_frame -> width;
          m_frame_height = m_frame -> height;
          //inf("Image Size: %d x %d \t TASK: STREAM",frame_width, frame_height);
        }
      }
      //! Main loop.
      void
      onMain(void)
      {
        inicValues();
        inicTCP(0);
        inicCapture();    
        inicTCP(1);
        #if defined(DUNE_USING_RASPICAMCV)
        m_flag_capture = 2;
        #endif
        while (!stopping())
        {
          while(m_client.size >= 0 && !stopping())
          {
            m_t1=(double)cvGetTickCount();
            waitForMessages(0.01);
            #if defined(DUNE_USING_RASPICAMCV)
            while(m_flag_capture == 0 && !stopping());
            m_flag_capture = 2;
            m_img = raspiCamCvQueryFrame(m_capture);
            m_flag_capture = 1;
            cvReleaseImage( &m_frame );
            if (m_frame == 0 )
              m_frame = cvCreateImage ( cvSize(m_inic_width, m_inic_height), m_img -> depth, m_img -> nChannels);
            cvResize(m_img, m_frame);
            #else
            m_frame = cvQueryFrame( m_capture );
            #endif
            
            if ( !m_capture )
            {
              err(DTR("ERROR GRAB IMAGE"));
            }            
       
            if (m_zlib_data == NULL)
              m_zlib_data = cvCreateImage(cvGetSize(m_frame),8,3);

            m_dsize = m_frame->imageSize + (m_frame->imageSize * 0.1f) + 20;
            m_result = compress2((unsigned char *)m_zlib_data->imageData, &m_dsize, (const unsigned char 
            *)m_frame->imageData, m_frame->imageSize, 5);
       
            if(m_result != Z_OK)
              err(DTR("Compress error occured!"));
            //send data size
            sprintf(m_buffer,"%ld\n",m_dsize);
            dispatchToClients(m_buffer, strlen(m_buffer));
            if(m_client.size == -1)
              break;

            m_ok_send = 0;
            m_tam_ok=0;
            m_cnt_refresh_sync=0;
            while(m_ok_send == 0)
            {
              ///recv data for sync and debug
              m_client = checkClientSockets();
              if (m_client.size == -1)
                err(DTR("ERROR reading of socket"));
              m_tam_ok = strlen(m_client.data);
              if(m_tam_ok == 1)
              {    
                m_ok_send = atoi(m_client.data);
              }
              
              m_cnt_refresh_sync++;
              if (m_cnt_refresh_sync > 4)
                m_ok_send = 1;
            }

            //send data image
            dispatchToClients(m_zlib_data->imageData, m_dsize);
            if(m_client.size == -1)
              break;

            sprintf(m_buffer,"(TCP) LAT: %f # LON: %f # ALT: %.2f m\n", m_lat, m_lon, m_heig);
            dispatchToClients(m_buffer, strlen(m_buffer));
            if(m_client.size == -1)
              break;

            cvReleaseImage(&m_zlib_data);
            #if defined(DUNE_USING_RASPICAMCV)
            //save_video(frame,1);
            #endif
            m_t2=(double)cvGetTickCount();
            while(((m_t2-m_t1)/(cvGetTickFrequency()*1000.))<(1000/11))
            {
              m_t2=(double)cvGetTickCount();
            }
          }
          if(!stopping())
          {
            #if defined(DUNE_USING_RASPICAMCV)
            //raspiCamCvReleaseCapture( &capture );
            #else
            cvReleaseCapture(&m_capture);
            #endif
            war(DTR("Restarting connection TCP-IP"));
            inicTCP(0);
            inicTCP(1);
            m_client.size = 0;
            #if defined(DUNE_USING_RASPICAMCV)
            //inic_Capture();
            #else
            inicCapture();
            #endif
          }
        }
        #if defined(DUNE_USING_RASPICAMCV)
        raspiCamCvReleaseCapture( &m_capture );
        //saveVideo(frame,0);
        #else
        cvReleaseCapture(&m_capture);
        #endif
      }
    };
  }
}
DUNE_TASK
