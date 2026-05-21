//
// Created by createc on 09/09/2020.
//

#ifndef PHDS_GEGI_DRIVER_TCP_EVENT_READER_HPP
#define PHDS_GEGI_DRIVER_TCP_EVENT_READER_HPP

#include <phds_gegi_driver/data_structures.hpp>

#include <boost/asio.hpp>

#include <algorithm> // copy
#include <array> // array
#include <atomic> // atomic
#include <functional> // function, placeholders
#include <iostream> // cerr, cout, endl
#include <iterator> // begin, end
#include <mutex> // mutex
#include <stdexcept> // runtime_error
#include <string> // string
#include <thread> // thread
#include <variant> // holds_alternative, variant

namespace phds_gegi_driver::socket_comms {

    using ComptonEvent = std::variant<compton_events::Event1Site, compton_events::Event2Site>;

    using Callback1Site = std::function<void(const compton_events::Event1Site &)>;
    using Callback2Site = std::function<void(const compton_events::Event2Site &)>;

    struct RunInfo {
        double real_time_sec = 0.0;
        double live_time_sec = 0.0;
        double dead_time_percent = 0.0;
        double count_rate_hz = 0.0;
    };

    struct DetectorInfo {
        std::string serial_number;
        double detector_temp_kelvin = 0.0;
        int detector_bias_status = 0;
        int line_power_status = 0;
        int batt1_percent = 0;
        int batt2_percent = 0;
    };

    class TcpEventReader {
    public:
        TcpEventReader(Callback1Site callback_1_site, Callback2Site callback);

        void connect(const std::string &ip, const std::string &port);
        void startListening();

        // Remote control commands (single-byte ASCII)
        bool sendStartAcquisition();    // 'g'
        bool sendStopAcquisition();     // 's'
        bool sendClearData();           // 'c'
        bool sendClearDataAndWindows(); // 'x'
        bool sendToggleBiasMode();      // 'b'
        bool sendTimedAcquisition(char preset_cmd); // '1'..'5'
        RunInfo getRunInfo();           // 'r' - requests and parses real/live/dead times + count rate
        DetectorInfo getDetectorInfo(); // 'd' - requests detector status info

        void stopListening();
        void disconnect();

        ~TcpEventReader();

    private:
        void monitorSocket();
        bool sendCommand(char cmd);  // Low-level send
        static RunInfo parseRunInfo(const std::string &response);

        boost::asio::io_service io_service_;

        Callback1Site callback_1_site_;

        Callback2Site callback_2_site_;

        std::thread monitoring_thread_;

        boost::asio::ip::tcp::socket socket_;
        std::mutex socket_mutex_;     // Protect socket from concurrent sends
        std::mutex response_mutex_;   // Held during command response reads; monitorSocket waits on this

        std::atomic<bool> running_ {false};

        static constexpr int PACKET_SIZE_BYTES_ = 92;  // Legacy: max packet size (Compton events)
    };
} // namespace phds_gegi_driver::socket_comms

#endif //PHDS_GEGI_DRIVER_TCP_EVENT_READER_HPP
