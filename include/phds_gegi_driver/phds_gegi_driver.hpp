//
// Created by createc on 10/09/2020.
//

#ifndef PHDS_GEGI_DRIVER_PHDS_GEGI_DRIVER_HPP
#define PHDS_GEGI_DRIVER_PHDS_GEGI_DRIVER_HPP

#include <phds_gegi_driver/data_structures.hpp>
#include <socket_comms/tcp_event_reader.hpp>

#include <radiation_detector_msgs/ComptonEvent.h>
#include <std_msgs/Float64.h>
#include <phds_gegi_driver/StartAcquisition.h>
#include <phds_gegi_driver/StopAcquisition.h>
#include <phds_gegi_driver/ClearData.h>
#include <phds_gegi_driver/GetRunInfo.h>
#include <phds_gegi_driver/GetDetectorInfo.h>
#include <phds_gegi_driver/ToggleBiasMode.h>
#include <phds_gegi_driver/StartTimedAcquisition.h>

#include <ros/ros.h>

#include <cstdlib> // exit
#include <functional> // bind
#include <string> // string

namespace phds_gegi_driver {
    class PhdsGegiDriver {
    public:
        PhdsGegiDriver(ros::NodeHandle& node, ros::NodeHandle& node_private);

    private:
        void process1SiteEvent(const compton_events::Event1Site &event);

        void process2SiteEvent(const compton_events::Event2Site &event);

        void publishEnergy(double energy_kev);

        // Remote control service callbacks
        bool handleStartAcquisition(phds_gegi_driver::StartAcquisition::Request &req,
                                    phds_gegi_driver::StartAcquisition::Response &res);
        bool handleStopAcquisition(phds_gegi_driver::StopAcquisition::Request &req,
                                   phds_gegi_driver::StopAcquisition::Response &res);
        bool handleClearData(phds_gegi_driver::ClearData::Request &req,
                            phds_gegi_driver::ClearData::Response &res);
        bool handleGetRunInfo(phds_gegi_driver::GetRunInfo::Request &req,
                             phds_gegi_driver::GetRunInfo::Response &res);
        bool handleGetDetectorInfo(phds_gegi_driver::GetDetectorInfo::Request &req,
                      phds_gegi_driver::GetDetectorInfo::Response &res);
        bool handleToggleBiasMode(phds_gegi_driver::ToggleBiasMode::Request &req,
                     phds_gegi_driver::ToggleBiasMode::Response &res);
        bool handleStartTimedAcquisition(phds_gegi_driver::StartTimedAcquisition::Request &req,
                                         phds_gegi_driver::StartTimedAcquisition::Response &res);

        unsigned int event_sequence_id_ = 0;

        socket_comms::TcpEventReader tcp_event_reader_;

        ros::Publisher event_publisher_;
        ros::Publisher energy_publisher_;

        // Remote control service servers
        ros::ServiceServer start_acq_service_;
        ros::ServiceServer stop_acq_service_;
        ros::ServiceServer clear_data_service_;
        ros::ServiceServer run_info_service_;
        ros::ServiceServer detector_info_service_;
        ros::ServiceServer toggle_bias_service_;
        ros::ServiceServer timed_acq_service_;

        std::string detector_frame_;

    };
}

#endif //PHDS_GEGI_DRIVER_PHDS_GEGI_DRIVER_HPP
