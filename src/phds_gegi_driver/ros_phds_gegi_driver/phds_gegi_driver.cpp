//
// Created by createc on 10/09/2020.
//

#include <phds_gegi_driver/phds_gegi_driver.hpp>

#include <cmath>

namespace { // Anon
    template<typename T>
    void getParam(const ros::NodeHandle& node, const std::string& name, T& value)
    {
        if(!node.getParam(name, value))
            ROS_WARN_STREAM("PhdsGegiDriver: Failed to get \"" << name << "\" parameter, using default value: " << value);
    }
}

namespace phds_gegi_driver {
    PhdsGegiDriver::PhdsGegiDriver(ros::NodeHandle& node, ros::NodeHandle& node_private)
            : tcp_event_reader_(std::bind(&PhdsGegiDriver::process1SiteEvent, this, std::placeholders::_1),
                                std::bind(&PhdsGegiDriver::process2SiteEvent, this, std::placeholders::_1))
    {
        ROS_INFO("Starting ROS PHDS GeGI driver");

        ros::NodeHandle& n = node;
        ros::NodeHandle& pn = node_private;

        std::string gegi_ip;
        std::string gegi_port = "27015";
        std::string event_topic = "/compton_event";
        std::string energy_topic = "/energy_deposit";

        getParam(pn, "gegi_ip", gegi_ip);
        getParam(pn, "gegi_port", gegi_port);
        getParam(pn, "event_topic", event_topic);
        getParam(pn, "energy_topic", energy_topic);
        getParam(pn, "detector_frame", detector_frame_);

        constexpr static int OUTPUT_BUFFER_SIZE = 100;
        event_publisher_ = n.advertise<radiation_detector_msgs::ComptonEvent>(event_topic, OUTPUT_BUFFER_SIZE);
        energy_publisher_ = n.advertise<std_msgs::Float64>(energy_topic, OUTPUT_BUFFER_SIZE);

        try {
            tcp_event_reader_.connect(gegi_ip, gegi_port);
            tcp_event_reader_.startListening();
        } catch (std::exception &error) {
            ROS_ERROR_STREAM("PhdsGegiDriver: Could not start TCP IP socket: " << error.what());
            exit(1);
        }

        // Advertise remote control services
        start_acq_service_ = n.advertiseService("detector/start_acquisition",
                                                 &PhdsGegiDriver::handleStartAcquisition, this);
        stop_acq_service_ = n.advertiseService("detector/stop_acquisition",
                                                &PhdsGegiDriver::handleStopAcquisition, this);
        clear_data_service_ = n.advertiseService("detector/clear_data",
                                                  &PhdsGegiDriver::handleClearData, this);
        run_info_service_ = n.advertiseService("detector/get_run_info",
                                                &PhdsGegiDriver::handleGetRunInfo, this);
        detector_info_service_ = n.advertiseService("detector/get_detector_info",
                                &PhdsGegiDriver::handleGetDetectorInfo, this);
        toggle_bias_service_ = n.advertiseService("detector/toggle_bias_mode",
                              &PhdsGegiDriver::handleToggleBiasMode, this);
        timed_acq_service_ = n.advertiseService("detector/start_timed_acquisition",
                             &PhdsGegiDriver::handleStartTimedAcquisition, this);
        ROS_INFO("Detector remote control services advertised");

        ROS_INFO("ROS PHDS GeGI driver initialised");
    }

    void PhdsGegiDriver::process1SiteEvent(const compton_events::Event1Site &event) {
        publishEnergy(event.compton_interaction.avg_energy);
    }

    void PhdsGegiDriver::process2SiteEvent(const compton_events::Event2Site &event) {

        radiation_detector_msgs::ComptonEvent ros_event;

        ros_event.header.stamp = ros::Time::now();
        ros_event.header.frame_id = detector_frame_;
        ros_event.header.seq = event_sequence_id_;
        ++event_sequence_id_;

        constexpr static double MM_TO_M = 0.001;
        constexpr static double MEV_TO_KEV = 1.0;

        ros_event.energy_kev_1 = event.compton_interaction_1.avg_energy * MEV_TO_KEV;
        ros_event.energy_kev_2 = event.compton_interaction_2.avg_energy * MEV_TO_KEV;

        // Switching coordinates to ROS frame (REP-105)
        ros_event.reading_location_1.x = event.compton_interaction_1.site.z * MM_TO_M;
        ros_event.reading_location_1.y = event.compton_interaction_1.site.x * MM_TO_M;
        ros_event.reading_location_1.z = -event.compton_interaction_1.site.y * MM_TO_M;

        ros_event.reading_location_2.x = event.compton_interaction_2.site.z * MM_TO_M;
        ros_event.reading_location_2.y = event.compton_interaction_2.site.x * MM_TO_M;
        ros_event.reading_location_2.z = -event.compton_interaction_2.site.y * MM_TO_M;

        ros_event.cone_angle = event.compton_angle;
        ros_event.cone_angle_uncertainty = event.delta_compton_angle;

        event_publisher_.publish(ros_event);

        // Publish summed energy (full-energy deposition) for spectrum
        publishEnergy(ros_event.energy_kev_1 + ros_event.energy_kev_2);
    }

    void PhdsGegiDriver::publishEnergy(double energy_kev) {
        std_msgs::Float64 msg;
        msg.data = energy_kev;
        energy_publisher_.publish(msg);
    }

    bool PhdsGegiDriver::handleStartAcquisition(phds_gegi_driver::StartAcquisition::Request &req,
                                                phds_gegi_driver::StartAcquisition::Response &res) {
        ROS_INFO("Received start_acquisition request");
        res.success = tcp_event_reader_.sendStartAcquisition();
        res.message = res.success ? "Start acquisition command sent" : "Failed to send start acquisition command";
        return true;
    }

    bool PhdsGegiDriver::handleStopAcquisition(phds_gegi_driver::StopAcquisition::Request &req,
                                               phds_gegi_driver::StopAcquisition::Response &res) {
        ROS_INFO("Received stop_acquisition request");
        res.success = tcp_event_reader_.sendStopAcquisition();
        res.message = res.success ? "Stop acquisition command sent" : "Failed to send stop acquisition command";
        return true;
    }

    bool PhdsGegiDriver::handleClearData(phds_gegi_driver::ClearData::Request &req,
                                        phds_gegi_driver::ClearData::Response &res) {
        ROS_INFO("Received clear_data request");
        res.success = tcp_event_reader_.sendClearData();
        res.message = res.success ? "Clear data command sent" : "Failed to send clear data command";
        return true;
    }

    bool PhdsGegiDriver::handleGetRunInfo(phds_gegi_driver::GetRunInfo::Request &req,
                                         phds_gegi_driver::GetRunInfo::Response &res) {
        ROS_INFO("Received get_run_info request");
        auto run_info = tcp_event_reader_.getRunInfo();
        res.real_time_sec = run_info.real_time_sec;
        res.live_time_sec = run_info.live_time_sec;
        res.dead_time_percent = run_info.dead_time_percent;
        res.count_rate_hz = run_info.count_rate_hz;
        const bool valid = std::isfinite(res.real_time_sec)
                           && std::isfinite(res.live_time_sec)
                           && std::isfinite(res.dead_time_percent)
                           && std::isfinite(res.count_rate_hz)
                           && res.real_time_sec >= 0.0
                           && res.live_time_sec >= 0.0
                           && res.dead_time_percent >= -100.0
                           && res.dead_time_percent <= 100.0
                           && res.count_rate_hz >= 0.0;

        res.success = valid;
        res.message = valid ? "Run info retrieved" : "Run info response invalid or timed out";
        return true;
    }

    bool PhdsGegiDriver::handleGetDetectorInfo(phds_gegi_driver::GetDetectorInfo::Request &req,
                                              phds_gegi_driver::GetDetectorInfo::Response &res) {
        ROS_INFO("Received get_detector_info request");
        auto detector_info = tcp_event_reader_.getDetectorInfo();
        res.serial_number = detector_info.serial_number;
        res.detector_temp_kelvin = detector_info.detector_temp_kelvin;
        res.detector_bias_status = detector_info.detector_bias_status;
        res.line_power_status = detector_info.line_power_status;
        res.batt1_percent = detector_info.batt1_percent;
        res.batt2_percent = detector_info.batt2_percent;
        const bool valid = !res.serial_number.empty()
                           && std::isfinite(res.detector_temp_kelvin)
                           && res.detector_temp_kelvin >= -50.0
                           && res.detector_temp_kelvin <= 200.0
                           && (res.detector_bias_status == 0 || res.detector_bias_status == 1)
                           && (res.line_power_status == 0 || res.line_power_status == 1)
                           && res.batt1_percent >= 0
                           && res.batt1_percent <= 100
                           && res.batt2_percent >= 0
                           && res.batt2_percent <= 100;

        res.success = valid;
        res.message = valid ? "Detector info retrieved" : "Detector info response invalid or timed out";
        return true;
    }

    bool PhdsGegiDriver::handleToggleBiasMode(phds_gegi_driver::ToggleBiasMode::Request &req,
                                             phds_gegi_driver::ToggleBiasMode::Response &res) {
        ROS_INFO("Received toggle_bias_mode request");
        res.success = tcp_event_reader_.sendToggleBiasMode();
        res.message = res.success ? "Bias toggle command sent" : "Failed to send bias toggle command";
        return true;
    }

    bool PhdsGegiDriver::handleStartTimedAcquisition(phds_gegi_driver::StartTimedAcquisition::Request &req,
                                                     phds_gegi_driver::StartTimedAcquisition::Response &res) {
        ROS_INFO_STREAM("Received start_timed_acquisition request: " << req.duration_minutes << " minutes");

        // Map duration to GeGI preset command character ('1'=5min, '2'=10min, etc.)
        char cmd;
        switch (req.duration_minutes) {
            case 5:  cmd = '1'; break;
            case 10: cmd = '2'; break;
            case 15: cmd = '3'; break;
            case 20: cmd = '4'; break;
            case 25: cmd = '5'; break;
            default:
                res.success = false;
                res.message = "Unsupported duration. Valid values: 5, 10, 15, 20, 25 minutes";
                return true;
        }

        res.success = tcp_event_reader_.sendTimedAcquisition(cmd);
        res.message = res.success
            ? std::to_string(req.duration_minutes) + "-minute acquisition started"
            : "Failed to send timed acquisition command";
        return true;
    }
}
