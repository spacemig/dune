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

//OpenCV headers
#include <opencv2/opencv.hpp>

//Enable(1) / disable(0) support for Raspicam
#define raspicam_on 0

//RaspiCAM headers
#if raspicam_on == 1
//RaspiCAM headers
#include "RaspiCamCV.h"
RaspiCamCvCapture* capture;
int flag_capture = 0;
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
      #if raspicam_on == 1
      //RaspiCam config
      RASPIVID_CONFIG * config;
      //Capture struct - OpenCV/RaspiCAM
      //RaspiCamCvCapture* capture;
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
      //IpCam Addresses
      const char* ipcam_addresses;
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
      //Buffer of tcp sender
      char buffer[70];
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
      //Height
      float heig;
      //Start Time - Save func.
      double t1;
      //End Time - Save func.
      double t2;
      // Socket handle.
      TCPSocket* m_sock;
      // I/O Multiplexer.
      Poll m_poll;
      // Clients.
      std::list<TCPSocket*> m_clients;
      //State of TCPsocket
      int stateTcpIni;
      
      // Client data.
      struct Client
      {
        int size;       // size of data received.
        char data[16];  // Data.
      };
      Client client_;

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
          ipcam_addresses = m_args.ipcam[0].c_str();
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
              client_.size = -1;
              (void)e;
              m_poll.remove(*(*itr));
              delete *itr;
              itr = m_clients.erase(itr);
              continue;
          }
          catch (std::runtime_error& e)
          {
            client_.size = -1;
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
        double lat_rad = msg->lat;
        double lon_rad = msg->lon;
        heig = (msg->height) - (msg->z);
        //LAT and LON rad
        //LAT and LON deg
        double lat_deg = lat_rad*(180/M_PI);
        double lon_deg = lon_rad*(180/M_PI);
        //Offset (m)
        double offset_n = msg->x;
        double offset_e = msg->y;
        //Lat and Lon final
        lat = lat_deg + (180/M_PI)*(offset_e/6378137);
        lon = lon_deg + (180/M_PI)*(offset_n/6378137)/cos(lat_deg);
        //inf("Lat = %f   Lon = %f", lat, lon);
      }
      //! Initialize Values
      void 
      InicValues(void)
      {
        stateTcpIni = 0;
        client_.size = 0;
        flag_stat_video = 0;
        
        #if raspicam_on == 1
        config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
        inic_width = 160;
        inic_height = 120;
        config->width = 640;
        config->height = 480;
        config->bitrate = 0; // zero: leave as default
        //config->framerate = 20;
        config->monochrome = 0;
        #else
        inic_width = 160;
        inic_height = 120;
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
          inf(DTR("Waiting from client"));
          while(stateTcpIni == 0 && !stopping())
          {
            if (m_poll.poll(0.01) && stateTcpIni == 0)
            {
              checkMainSocket();
              stateTcpIni = 1;
            }
          }
          inf(DTR("Connection TCP-IP ON"));
        }
        else
        {
          //Send info size image over tcp
          inf(DTR("Sending Data image info"));
          sprintf(buffer,"%d\n",frame_width);
          dispatchToClients(buffer, strlen(buffer));
          
          sprintf(buffer,"%d\n",frame_height);
          dispatchToClients(buffer, strlen(buffer));

          inf(DTR("Image Size: %dx%d"), frame_width, frame_height);
          stateTcpIni = 0;
        }
      }
      
      #if raspicam_on == 1
      /* Save Video Frame Result */
      void save_video(IplImage* image, bool parameter)
      {
        if (flag_stat_video == 0 && parameter == 1)
        {
          time_acquisition();
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
          #if raspicam_on == 0
          cvGrabFrame(capture);
          #endif
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
        capture = cvCaptureFromFile(ipcam_addresses);
        #endif
        
        while ( capture  == 0 && !stopping())
        {
          err(DTR("ERROR OPEN CAM\n"));
          #if raspicam_on == 1
          capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture2(0, config);
          #else
          capture = cvCaptureFromFile(ipcam_addresses);
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
          //inf("Image Size: %d x %d \t TASK: STREAM",frame_width, frame_height);
        }
      }
      //! Main loop.
      void
      onMain(void)
      {
        InicValues();
        InicTCP(0);
        inic_Capture();    
        InicTCP(1);
        #if raspicam_on == 1
        flag_capture = 2;
        #endif
        while (!stopping())
        {
          while(client_.size >= 0 && !stopping())
          {
            t1=(double)cvGetTickCount();
            waitForMessages(0.01);
            #if raspicam_on == 1
            while(flag_capture == 0 && !stopping());
            flag_capture = 2;
            img = raspiCamCvQueryFrame(capture);
            flag_capture = 1;
            cvReleaseImage( &frame );
            if (frame == 0 )
              frame = cvCreateImage ( cvSize(inic_width, inic_height), img -> depth, img -> nChannels);
            cvResize(img, frame);
            #else
            frame = cvQueryFrame( capture );
            #endif
            
            if ( !capture )
            {
              err(DTR("ERROR GRAB IMAGE"));
            }            
       
            if (zlib_data == NULL)
              zlib_data = cvCreateImage(cvGetSize(frame),8,3);

            dsize = frame->imageSize + (frame->imageSize * 0.1f) + 20;
            result = compress2((unsigned char *)zlib_data->imageData, &dsize, (const unsigned char 
            *)frame->imageData, frame->imageSize, 5);
       
            if(result != Z_OK)
              err(DTR("Compress error occured!"));
            //send data size
            sprintf(buffer,"%ld\n",dsize);
            dispatchToClients(buffer, strlen(buffer));
            if(client_.size == -1)
              break;

            ok_send = 0;
            tam_ok=0;
            cnt_refresh_sync=0;
            while(ok_send == 0)
            {
              ///recv data for sync and debug
              client_ = checkClientSockets();
              if (client_.size == -1)
                err(DTR("ERROR reading of socket"));
              tam_ok = strlen(client_.data);
              if(tam_ok == 1)
              {    
                ok_send = atoi(client_.data);
              }
              
              cnt_refresh_sync++;
              if (cnt_refresh_sync > 4)
                ok_send = 1;
            }

            //send data image
            dispatchToClients(zlib_data->imageData, dsize);
            if(client_.size == -1)
              break;

            sprintf(buffer,"(TCP) LAT: %f # LON: %f # ALT: %.2f m\n", lat, lon, heig);
            dispatchToClients(buffer, strlen(buffer));
            if(client_.size == -1)
              break;

            cvReleaseImage(&zlib_data);
            #if raspicam_on == 1
            //save_video(frame,1);
            #endif
            t2=(double)cvGetTickCount();
            while(((t2-t1)/(cvGetTickFrequency()*1000.))<(1000/11))
            {
              t2=(double)cvGetTickCount();
            }
          }
          if(!stopping())
          {
            #if raspicam_on == 1
            //raspiCamCvReleaseCapture( &capture );
            #else
            cvReleaseCapture(&capture);
            #endif
            war(DTR("Restarting connection TCP-IP"));
            InicTCP(0);
            InicTCP(1);
            client_.size = 0;
            #if raspicam_on == 0
            inic_Capture();
            #endif
          }
        }
        #if raspicam_on == 1
        raspiCamCvReleaseCapture( &capture );
//        save_video(frame,0);
        #else
        cvReleaseCapture(&capture);
        #endif
      }
    };
  }
}
DUNE_TASK