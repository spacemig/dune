//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Miguel Nunes                                                     *
//***************************************************************************

// ISO C++ 98 headers.
#include <vector>
#include <cmath>
#include <queue>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// MAVLink headers.
//#include <mavlink/ardupilotmega/mavlink.h>

//DJI OSDK Library Headers
#include "dji_vehicle.hpp"
#include "dji_status.hpp"
#include "dji_telemetry.hpp"
#include "dji_linux_helpers.hpp"
#include "flight_control_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

namespace Control
{
  namespace UAV
  {
    namespace DJI_Autopilot
    {
      using DUNE_NAMESPACES;

      //! %Task arguments.
      struct Arguments
      {
        //! Communications timeout
        uint8_t comm_timeout;
        //! TCP Port
//        uint16_t TCP_port;
        //! TCP Address
//        Address TCP_addr;
        //! Telemetry Rate
        uint8_t trate;
        // Serial port device.
        std::string uart_dev;
        // Serial port baud rate.
        unsigned uart_baud;
      };


      struct Task: public DUNE::Tasks::Task
      {
        //! Task arguments.
        Arguments m_args;
        double m_last_pkt_time;
        uint8_t m_buf[512];
        //! TCP socket
//        Network::TCPSocket* m_TCP_sock;
        //! DJI serial
//        DJI::onboardSDK::LinuxSerialDevice* serialDevice;
//        DJI::onboardSDK::CoreAPI* api;
//        DJI::onboardSDK::Flight* flight;
//        DJI::onboardSDK::WayPoint* waypointObj;

        //! Estimated state message.
        IMC::EstimatedState m_estate;
        bool m_error_missing, m_esta_ext;
        //! External control
        bool m_external;

        Task(const std::string& name, Tasks::Context& ctx):
          Tasks::Task(name, ctx)
        {

//          param("TCP - Port", m_args.TCP_port)
//              .defaultValue("5760")
//              .description("Port for connection to Ardupilot");

//          param("TCP - Address", m_args.TCP_addr)
//              .defaultValue("127.0.0.1")
//              .description("Address for connection to Ardupilot");

          param("Communications Timeout", m_args.comm_timeout)
              .minimumValue("1")
              .maximumValue("60")
              .defaultValue("10")
              .units(Units::Second)
              .description("Ardupilot communications timeout");

          // Define configuration parameters.
          param("Serial Port - Device", m_args.uart_dev)
              .defaultValue("/dev/ttyUSB0")
              .description("Serial port device used to communicate with the sensor");

          param("Serial Port - Baud Rate", m_args.uart_baud)
              .defaultValue("115200")
              .description("Serial port baud rate");

          // Setup packet handlers

          // Setup processing of IMC messages
          bind<DesiredPath>(this);
          bind<DesiredRoll>(this);
          bind<DesiredZ>(this);
          bind<DesiredVerticalRate>(this);
          bind<DesiredSpeed>(this);
          bind<DesiredControl>(this);
          bind<IdleManeuver>(this);
          bind<ControlLoops>(this);
          bind<VehicleMedium>(this);
          bind<VehicleState>(this);
          bind<SimulatedState>(this);
          bind<DevCalibrationControl>(this);
          bind<AutopilotMode>(this);

          //! Misc. initialization
          m_last_pkt_time = 0; //! time of last packet from Ardupilot
          m_estate.clear();
        }

        void
        onResourceRelease(void)
        {
//          Memory::clear(m_TCP_sock);
        }

        void
        onResourceAcquisition(void)
        {
          inf("onResourceAcquisition");
          openConnection();
        }

        void
        onUpdateParameters(void)
        {
        }

        void
        setupRate(uint8_t rate)
        {
        }

        void
        consume(const IMC::ControlLoops* cloops)
        {
        }

        void
        consume(const IMC::DesiredRoll* d_roll)
        {
        }

        void
        consume(const IMC::DesiredControl* d_acc)
        {
        }

        void
        consume(const IMC::DesiredZ* d_z)
        {
        }

        void
        consume(const IMC::DesiredVerticalRate* d_vr)
        {
        }

        void
        consume(const IMC::DesiredSpeed* d_speed)
        {
        }

        //! Message for GUIDED/AUTO control (using Ardupilot's controllers)
        void
        consume(const IMC::DesiredPath* path)
        {
        }

        void
        takeoff_copter(const IMC::DesiredPath* dpath)
        {
        }

        void
        loiterHere(void)
        {
        }

        void
        consume(const IMC::IdleManeuver* idle)
        {
        }

        void
        consume(const IMC::VehicleMedium* vm)
        {
        }

        void
        consume(const IMC::VehicleState* msg)
        {
//            msg->
        }

        //! Used for HITL simulations
        void
        consume(const IMC::SimulatedState* sim_state)
        {
        }

        void
        consume(const IMC::DevCalibrationControl* msg)
        {
        }

        void
        consume(const IMC::AutopilotMode* msg)
        {
        }

        void
        onMain(void)
        {
//          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

          while (!stopping())
            {
              setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

              // Handle data
              if (1) // if serial DJI
                {
                  handleDjiData();
                }
              else
                {
                  Time::Delay::wait(0.5);
                  openConnection();
                }

              if (!m_error_missing)
                {
                  if (m_external)
                    {
                      if (!m_esta_ext)
                        {
                          setEntityState(IMC::EntityState::ESTA_NORMAL, "External Control");
                          m_esta_ext = true;
                        }
                    }
                  else// if (getEntityState() != IMC::EntityState::ESTA_NORMAL)
                    {
                      setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
                      m_esta_ext = false;
                    }
                }

              // Handle IMC messages from bus
              consumeMessages();
              waitForMessages(0.5);
            }
        }

        bool
        poll(double timeout)
        {
//          if (m_TCP_sock != NULL)
//            return Poll::poll(*m_TCP_sock, timeout);

          return false;
        }

        int
        sendData(uint8_t* bfr, int size)
        {
//          if (m_TCP_sock)
//            {
//              trace("Sending something");
//              return m_TCP_sock->write((char*)bfr, size);
//            }
          return 0;
        }

        int
        receiveData(uint8_t* buf, size_t blen)
        {
//          if (m_TCP_sock)
//            {
//              try
//              {
//                return m_TCP_sock->read(buf, blen);
//              }
//              catch (std::runtime_error& e)
//              {
//                err("%s", e.what());
//                war(DTR("Connection lost, retrying..."));
//                Memory::clear(m_TCP_sock);

//                m_TCP_sock = new Network::TCPSocket;
//                m_TCP_sock->connect(m_args.TCP_addr, m_args.TCP_port);
//                return 0;
//              }
//            }
          return 0;
        }

        void
        openConnection(void)
        {

          try
          {
            //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.

            char* config_file[2] = {"","/home/miguel/rep18/hsfl-dji/UserConfig.txt"};
            LinuxSetup linuxEnvironment (2, config_file);
            Vehicle* vehicle = linuxEnvironment.getVehicle();


//            serialDevice = new DJI::onboardSDK::LinuxSerialDevice(m_args.uart_dev, m_args.uart_baud);
//            api = new DJI::onboardSDK::CoreAPI(serialDevice);
//            flight = new DJI::onboardSDK::Flight(api);
//            waypointObj = new DJI::onboardSDK::WayPoint(api);
            //            ackReturnData releaseControlStatus;

            //! Initializes the read thread and the call back thread.
            //LinuxThread readThread(api,2);

//            m_TCP_sock = new TCPSocket;
//            m_TCP_sock->connect(m_args.TCP_addr, m_args.TCP_port);
//            m_TCP_sock->setNoDelay(true);

            //! Enable non-blocking callback thread mechanism
            //            api->nonBlockingCBThreadEnable = true;

            //! Initializes the read thread and the call back thread.
//            LinuxThread readThread(api,2);
            //            LinuxThread CallbackThread(api,3);

            //! Setup
//            int setupStatus = setup(serialDevice, api, &readThread);

//            if (setupStatus == -1)
//              {
//                std::cout << "This program will exit now. \n";
//                exit(0);
//              }

            inf("1");

            //! Set broadcast Freq Defaults
//            unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
            usleep(500000);

            inf("2");

            //            takeControlNonBlocking(api);
            //            takeoffNonBlocking(flight);

            //            interactiveSpin(api, flight, waypointObj);

            int blockingTimeout = 1; //Seconds

            //! Monitored Takeoff
//            ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

            inf("3");

            //! Set broadcast Freq Defaults
//            broadcastAck = api->setBroadcastFreqDefaults(1);

            inf("4");

            //! If the aircraft took off, continue to do flight control tasks
//            if (takeoffStatus.status == 1)
//              {

//                std::cout << "Hello M100" << std::endl;

//              } else {
//                std::cout << "DAHHHH M100" << std::endl;
//              }

            inf("5");

            setupRate(m_args.trate);
            inf(DTR("Ardupilot interface initialized"));
          }
          catch (...)
          {
//            Memory::clear(m_TCP_sock);

            war(DTR("Connection failed, retrying..."));
            setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_COM_ERROR);
          }
        }

        void
        handleDjiData(void)
        {

          double now = Clock::get();
          int counter = 0;

          // simple test to dispatch data to the IMC bus (does it show on Neptus?)
          m_estate.lat = 10 ;
          m_estate.lon = 3  ;
          m_estate.height = 5 ;

          dispatch(m_estate);


          while (poll(0.01) && counter < 100)
            {
              counter++;

              int n = receiveData(m_buf, sizeof(m_buf));

              if (n < 0)
                {
                  debug("Receive error");
                  break;
                }

              now = Clock::get();

              inf("data rx (%d): %s", n, m_buf);

              for (int i = 0; i < n; i++)
                {
                  m_last_pkt_time = now;
                }
            }

          if (now - m_last_pkt_time >= m_args.comm_timeout)
            {
              if (!m_error_missing)
                {
                  setEntityState(IMC::EntityState::ESTA_ERROR, Status::CODE_MISSING_DATA);
                  m_error_missing = true;
                  m_esta_ext = false;
                }
            }
          else
            m_error_missing = false;
        }

      };

    }
  }
}

DUNE_TASK
