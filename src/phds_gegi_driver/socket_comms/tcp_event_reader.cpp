//
// Created by createc on 09/09/2020.
//

#include <socket_comms/tcp_event_reader.hpp>

#include <ros/ros.h>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

namespace phds_gegi_driver::socket_comms {

    namespace {
        bool readExactlyWithTimeout(boost::asio::ip::tcp::socket &socket,
                                    char *destination,
                                    size_t bytes_to_read,
                                    int timeout_ms) {
            size_t total_read = 0;
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

            while (total_read < bytes_to_read) {
                boost::system::error_code error;
                const size_t available = socket.available(error);
                if (error) {
                    std::cerr << "Socket available() failed: " << error.message() << std::endl;
                    return false;
                }

                if (available > 0) {
                    const size_t chunk_size = std::min(available, bytes_to_read - total_read);
                    const size_t bytes_read = socket.read_some(
                            boost::asio::buffer(destination + total_read, chunk_size), error);
                    if (error) {
                        std::cerr << "Socket read_some() failed: " << error.message() << std::endl;
                        return false;
                    }
                    total_read += bytes_read;
                    continue;
                }

                if (std::chrono::steady_clock::now() >= deadline) {
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            return true;
        }

        uint32_t byteSwap32(uint32_t value) {
            return ((value & 0x000000FFu) << 24u) |
                   ((value & 0x0000FF00u) << 8u) |
                   ((value & 0x00FF0000u) >> 8u) |
                   ((value & 0xFF000000u) >> 24u);
        }

        uint64_t byteSwap64(uint64_t value) {
            return ((value & 0x00000000000000FFull) << 56u) |
                   ((value & 0x000000000000FF00ull) << 40u) |
                   ((value & 0x0000000000FF0000ull) << 24u) |
                   ((value & 0x00000000FF000000ull) << 8u) |
                   ((value & 0x000000FF00000000ull) >> 8u) |
                   ((value & 0x0000FF0000000000ull) >> 24u) |
                   ((value & 0x00FF000000000000ull) >> 40u) |
                   ((value & 0xFF00000000000000ull) >> 56u);
        }

        uint32_t readU32(const char *data, bool big_endian) {
            uint32_t value = 0;
            std::memcpy(&value, data, sizeof(uint32_t));
            return big_endian ? byteSwap32(value) : value;
        }

        int32_t readI32(const char *data, bool big_endian) {
            return static_cast<int32_t>(readU32(data, big_endian));
        }

        double readF64(const char *data, bool big_endian) {
            uint64_t value = 0;
            std::memcpy(&value, data, sizeof(uint64_t));
            if (big_endian) {
                value = byteSwap64(value);
            }
            double output = 0.0;
            std::memcpy(&output, &value, sizeof(double));
            return output;
        }
    }

    TcpEventReader::TcpEventReader(Callback1Site callback_1_site, Callback2Site callback_2_site)
            : socket_(io_service_),
              callback_1_site_(std::move(callback_1_site)),
              callback_2_site_(std::move(callback_2_site)) {}

    void TcpEventReader::connect(const std::string &ip, const std::string &port) {
        using namespace boost::asio::ip;

        std::cout << "Connecting to GeGI at " + ip + ":" + port << "..." << std::endl;

        tcp::resolver resolver(io_service_);
        tcp::resolver::query query(ip, port);
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

        try {
            boost::asio::connect(socket_, endpoint_iterator);

        } catch (const std::exception &ex) {
            throw;
        }

        std::cout << "Connected to GeGI at " + ip + ":" + port << std::endl;
    }

    void TcpEventReader::startListening() {
        running_.store(true);

        if (monitoring_thread_.joinable()) {
            std::cerr << "Error: Trying to start listening on TCP socket when already listening" << std::endl;
            return;
        }

        monitoring_thread_ = std::thread(&TcpEventReader::monitorSocket, this);
    }

    void TcpEventReader::monitorSocket() {
        std::vector<char> stream_buffer;
        stream_buffer.reserve(4096);

        // Packet sizes per event type (from PHDS protocol documentation)
        // Type 0 (Single-Site):  4(type) + 8(time) + 8*4(x,y,z,e) = 44 bytes
        // Type 1 (Two-Site):     4(type) + 8(time) + 8*8(x1,y1,z1,e1,x2,y2,z2,e2) = 76 bytes
        // Type 2 (Compton):      4(type) + 8(time) + 8*10(x1,y1,z1,e1,x2,y2,z2,e2,angle,delta) = 92 bytes
        constexpr size_t SINGLE_SITE_BYTES = 44;
        constexpr size_t TWO_SITE_BYTES = 76;
        constexpr size_t COMPTON_BYTES = 92;

        auto packet_size_for_type = [](uint32_t event_type) -> size_t {
            switch (event_type) {
                case 0: return SINGLE_SITE_BYTES;
                case 1: return TWO_SITE_BYTES;
                case 2: return COMPTON_BYTES;
                default: return 0;
            }
        };

        auto read_u32_buf = [](const std::vector<char>& buf, size_t offset) {
            uint32_t value;
            std::memcpy(&value, buf.data() + offset, sizeof(uint32_t));
            return value;
        };

        auto read_double_buf = [](const std::vector<char>& buf, size_t offset) {
            double value;
            std::memcpy(&value, buf.data() + offset, sizeof(double));
            return value;
        };

        // Validate common fields at given base offset: event_type and first-site spatial/energy
        auto validate_header = [&](size_t base, size_t buf_size) -> bool {
            if (base + 44 > buf_size) return false;

            const uint32_t et = read_u32_buf(stream_buffer, base);
            if (et > 2) return false;

            const double x1 = read_double_buf(stream_buffer, base + 12);
            const double y1 = read_double_buf(stream_buffer, base + 20);
            const double z1 = read_double_buf(stream_buffer, base + 28);
            const double e1 = read_double_buf(stream_buffer, base + 36);

            if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(z1) || !std::isfinite(e1))
                return false;
            if (std::fabs(x1) > 80.0 || std::fabs(y1) > 80.0 || std::fabs(z1) > 80.0)
                return false;
            if (e1 < 0.05 || e1 > 5000.0)
                return false;

            return true;
        };

        while (running_.load()) {
            std::array<char, 1024> data_in{};
            boost::system::error_code error;
            size_t bytes_read = 0;

            {
                std::unique_lock<std::mutex> resp_lock(response_mutex_, std::try_to_lock);
                if (!resp_lock.owns_lock()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                const size_t available = socket_.available(error);
                if (error) {
                    std::cerr << "Socket available() failed: " << error.message() << std::endl;
                    running_.store(false);
                    return;
                }
                if (available == 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }

                std::lock_guard<std::mutex> lock(socket_mutex_);
                bytes_read = socket_.read_some(boost::asio::buffer(data_in), error);
            }

            if (error == boost::asio::error::eof) {
                std::cerr << "Connection lost" << std::endl;
                running_.store(false);
                return;
            } else if (error) {
                std::cerr << "Error reading from socket: " << error.message() << std::endl;
                continue;
            }

            if (bytes_read == 0) continue;

            stream_buffer.insert(stream_buffer.end(), data_in.begin(), data_in.begin() + static_cast<long long>(bytes_read));

            // Parse variable-length packets from the stream buffer
            while (stream_buffer.size() >= SINGLE_SITE_BYTES) {
                // Try to find a valid frame header at current position
                if (!validate_header(0, stream_buffer.size())) {
                    // Out of sync — discard one byte and retry
                    stream_buffer.erase(stream_buffer.begin());
                    continue;
                }

                const uint32_t event_type = read_u32_buf(stream_buffer, 0);
                const size_t pkt_size = packet_size_for_type(event_type);

                if (pkt_size == 0) {
                    // Unknown type, discard byte
                    stream_buffer.erase(stream_buffer.begin());
                    continue;
                }

                // Wait for full packet
                if (stream_buffer.size() < pkt_size) break;

                // Extract fields common to all types
                const double global_time = static_cast<double>(
                    *reinterpret_cast<const uint64_t*>(stream_buffer.data() + 4));
                const double x1 = read_double_buf(stream_buffer, 12);
                const double y1 = read_double_buf(stream_buffer, 20);
                const double z1 = read_double_buf(stream_buffer, 28);
                const double e1 = read_double_buf(stream_buffer, 36);

                if (event_type == 0) {
                    // Single-site event
                    compton_events::Event1Site ev{};
                    ev.global_time = global_time;
                    ev.compton_interaction.site.x = x1;
                    ev.compton_interaction.site.y = y1;
                    ev.compton_interaction.site.z = z1;
                    ev.compton_interaction.avg_energy = e1;
                    callback_1_site_(ev);

                } else if (event_type == 1) {
                    // Two-site event (no Compton angles)
                    const double x2 = read_double_buf(stream_buffer, 44);
                    const double y2 = read_double_buf(stream_buffer, 52);
                    const double z2 = read_double_buf(stream_buffer, 60);
                    const double e2 = read_double_buf(stream_buffer, 68);

                    // Treat as 1-site if second interaction is invalid
                    const bool site2_valid = std::isfinite(x2) && std::isfinite(y2) &&
                                             std::isfinite(z2) && std::isfinite(e2) &&
                                             std::fabs(x2) <= 80.0 && std::fabs(y2) <= 80.0 &&
                                             std::fabs(z2) <= 80.0 && e2 > 0.05 && e2 < 5000.0;

                    if (site2_valid) {
                        // Add each interaction energy separately to preserve spectral features
                        compton_events::Event1Site ev1{};
                        ev1.global_time = global_time;
                        ev1.compton_interaction.site.x = x1;
                        ev1.compton_interaction.site.y = y1;
                        ev1.compton_interaction.site.z = z1;
                        ev1.compton_interaction.avg_energy = e1;
                        callback_1_site_(ev1);

                        compton_events::Event1Site ev2{};
                        ev2.global_time = global_time;
                        ev2.compton_interaction.site.x = x2;
                        ev2.compton_interaction.site.y = y2;
                        ev2.compton_interaction.site.z = z2;
                        ev2.compton_interaction.avg_energy = e2;
                        callback_1_site_(ev2);
                    } else {
                        compton_events::Event1Site ev{};
                        ev.global_time = global_time;
                        ev.compton_interaction.site.x = x1;
                        ev.compton_interaction.site.y = y1;
                        ev.compton_interaction.site.z = z1;
                        ev.compton_interaction.avg_energy = e1;
                        callback_1_site_(ev);
                    }

                } else {
                    // Compton event (type 2) — full imaging event
                    const double x2 = read_double_buf(stream_buffer, 44);
                    const double y2 = read_double_buf(stream_buffer, 52);
                    const double z2 = read_double_buf(stream_buffer, 60);
                    const double e2 = read_double_buf(stream_buffer, 68);
                    const double compton_angle = read_double_buf(stream_buffer, 76);
                    const double delta_compton_angle = read_double_buf(stream_buffer, 84);

                    const bool site2_valid = std::isfinite(x2) && std::isfinite(y2) &&
                                             std::isfinite(z2) && std::isfinite(e2) &&
                                             std::fabs(x2) <= 80.0 && std::fabs(y2) <= 80.0 &&
                                             std::fabs(z2) <= 80.0 && e2 > 0.05 && e2 < 5000.0;
                    const bool angles_valid = std::isfinite(compton_angle) && std::isfinite(delta_compton_angle) &&
                                              compton_angle >= 0.0 && compton_angle <= 3.15 &&
                                              delta_compton_angle >= 0.0 && delta_compton_angle <= 1.0;

                    if (site2_valid && angles_valid) {
                        compton_events::Event2Site ev{};
                        ev.global_time = global_time;
                        ev.compton_interaction_1.site.x = x1;
                        ev.compton_interaction_1.site.y = y1;
                        ev.compton_interaction_1.site.z = z1;
                        ev.compton_interaction_1.avg_energy = e1;
                        ev.compton_interaction_2.site.x = x2;
                        ev.compton_interaction_2.site.y = y2;
                        ev.compton_interaction_2.site.z = z2;
                        ev.compton_interaction_2.avg_energy = e2;
                        ev.compton_angle = compton_angle;
                        ev.delta_compton_angle = delta_compton_angle;
                        callback_2_site_(ev);
                    } else {
                        // Demote to 1-site
                        compton_events::Event1Site ev{};
                        ev.global_time = global_time;
                        ev.compton_interaction.site.x = x1;
                        ev.compton_interaction.site.y = y1;
                        ev.compton_interaction.site.z = z1;
                        ev.compton_interaction.avg_energy = e1;
                        callback_1_site_(ev);
                    }
                }

                // Consume the packet from the buffer
                stream_buffer.erase(stream_buffer.begin(), stream_buffer.begin() + static_cast<long long>(pkt_size));
            }
        }
    }

    void TcpEventReader::stopListening() {
        running_.store(false);

        if (monitoring_thread_.joinable()) {
            // Closing the socket unblocks the listener thread's blocking read.
            // This path is used during shutdown, so reconnect is not expected.
            boost::system::error_code ignored_error;
            socket_.close(ignored_error);
            monitoring_thread_.join();
        }
    }

    void TcpEventReader::disconnect() {
        if (monitoring_thread_.joinable()) {
            throw std::runtime_error("Trying to disconnect from socket when monitoring data off it");
        }

        if (!socket_.is_open()) {
            std::cerr << "Trying to disconnect from closed socket" << std::endl;
            return;
        }

        boost::system::error_code error;

        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);
        if (error) {
            throw std::runtime_error("Could not shutdown socket");
        }

        socket_.close(error);
        if (error) {
            throw std::runtime_error("Could not close socket");
        }
    }

    bool TcpEventReader::sendCommand(char cmd) {
        if (!socket_.is_open()) {
            std::cerr << "Socket not open; cannot send command '" << cmd << "'" << std::endl;
            return false;
        }

        try {
            std::lock_guard<std::mutex> lock(socket_mutex_);
            boost::asio::write(socket_, boost::asio::buffer(&cmd, 1));
            std::cout << "Sent command '" << cmd << "' to detector" << std::endl;
            return true;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to send command '" << cmd << "': " << ex.what() << std::endl;
            return false;
        }
    }

    bool TcpEventReader::sendStartAcquisition() {
        return sendCommand('g');  // 'g' = Start Data Acquisition
    }

    bool TcpEventReader::sendStopAcquisition() {
        return sendCommand('s');  // 's' = Stop Data Acquisition
    }

    bool TcpEventReader::sendClearData() {
        return sendCommand('c');  // 'c' = Clear Data
    }

    bool TcpEventReader::sendClearDataAndWindows() {
        return sendCommand('x');  // 'x' = Clear Data and Energy Windows
    }

    bool TcpEventReader::sendToggleBiasMode() {
        return sendCommand('b');  // 'b' = Toggle Detector Mode (bias)
    }

    bool TcpEventReader::sendTimedAcquisition(char preset_cmd) {
        if (preset_cmd < '1' || preset_cmd > '5') {
            std::cerr << "Invalid timed acquisition preset command '" << preset_cmd << "'" << std::endl;
            return false;
        }

        return sendCommand(preset_cmd);
    }

    RunInfo TcpEventReader::getRunInfo() {
        RunInfo info;
        if (!socket_.is_open()) {
            std::cerr << "Socket not open; cannot request run info" << std::endl;
            return info;
        }

        try {
            std::lock_guard<std::mutex> resp_lock(response_mutex_);
            std::lock_guard<std::mutex> lock(socket_mutex_);
            char cmd = 'r';
            boost::asio::write(socket_, boost::asio::buffer(&cmd, 1));

            // Read response: int(4) + double(8) + double(8) + int(4) = 24 bytes
            std::vector<char> response(24);
            if (!readExactlyWithTimeout(socket_, response.data(), response.size(), 3000)) {
                std::cerr << "Timed out waiting for run info response" << std::endl;
            } else {
                auto decode_run_info = [&](bool big_endian) {
                    RunInfo decoded;
                    decoded.real_time_sec = static_cast<double>(readU32(response.data() + 0, big_endian));
                    decoded.live_time_sec = readF64(response.data() + 4, big_endian);
                    decoded.dead_time_percent = readF64(response.data() + 12, big_endian);
                    decoded.count_rate_hz = static_cast<double>(readU32(response.data() + 20, big_endian));
                    return decoded;
                };

                auto plausible_run_info = [](const RunInfo &candidate) {
                    return std::isfinite(candidate.real_time_sec)
                           && std::isfinite(candidate.live_time_sec)
                           && std::isfinite(candidate.dead_time_percent)
                           && std::isfinite(candidate.count_rate_hz)
                           && candidate.real_time_sec >= 0.0
                           && candidate.live_time_sec >= 0.0
                           && candidate.dead_time_percent >= -100.0
                           && candidate.dead_time_percent <= 100.0
                           && candidate.count_rate_hz >= 0.0
                           && candidate.real_time_sec <= 1.0e8
                           && candidate.live_time_sec <= 1.0e8
                           && candidate.count_rate_hz <= 1.0e8;
                };

                const auto little_endian = decode_run_info(false);
                const auto big_endian = decode_run_info(true);
                if (plausible_run_info(little_endian)) {
                    info = little_endian;
                } else if (plausible_run_info(big_endian)) {
                    info = big_endian;
                } else {
                    info = little_endian;
                }
            }

            std::cout << "Run Info: realTime=" << info.real_time_sec << "s, "
                      << "liveTime=" << info.live_time_sec << "s, "
                      << "deadTime=" << info.dead_time_percent << "%, "
                      << "countRate=" << info.count_rate_hz << " Hz" << std::endl;

        } catch (const std::exception &ex) {
            std::cerr << "Failed to get run info: " << ex.what() << std::endl;
        }

        return info;
    }

    DetectorInfo TcpEventReader::getDetectorInfo() {
        DetectorInfo info;
        if (!socket_.is_open()) {
            std::cerr << "Socket not open; cannot request detector info" << std::endl;
            return info;
        }

        try {
            std::lock_guard<std::mutex> resp_lock(response_mutex_);
            std::lock_guard<std::mutex> lock(socket_mutex_);
            char cmd = 'd';
            boost::asio::write(socket_, boost::asio::buffer(&cmd, 1));

            // Read response: char[4] + double(8) + int(4) + int(4) + int(4) + int(4) = 28 bytes
            std::vector<char> response(28);
            if (!readExactlyWithTimeout(socket_, response.data(), response.size(), 3000)) {
                std::cerr << "Timed out waiting for detector info response" << std::endl;
            } else {
                std::string serial(response.data(), 4);
                serial.erase(std::find(serial.begin(), serial.end(), '\0'), serial.end());

                auto decode_detector_info = [&](bool big_endian) {
                    DetectorInfo decoded;
                    decoded.serial_number = serial;
                    // Temperature stored as-is from device (field name is legacy)
                    decoded.detector_temp_kelvin = readF64(response.data() + 4, big_endian);
                    decoded.detector_bias_status = readI32(response.data() + 12, big_endian);
                    decoded.line_power_status = readI32(response.data() + 16, big_endian);
                    decoded.batt1_percent = readI32(response.data() + 20, big_endian);
                    decoded.batt2_percent = readI32(response.data() + 24, big_endian);
                    return decoded;
                };

                auto plausible_detector_info = [](const DetectorInfo &candidate) {
                    return !candidate.serial_number.empty()
                           && std::isfinite(candidate.detector_temp_kelvin)
                           && candidate.detector_temp_kelvin >= -50.0
                           && candidate.detector_temp_kelvin <= 200.0
                           && (candidate.detector_bias_status == 0 || candidate.detector_bias_status == 1)
                           && (candidate.line_power_status == 0 || candidate.line_power_status == 1)
                           && candidate.batt1_percent >= 0
                           && candidate.batt1_percent <= 100
                           && candidate.batt2_percent >= 0
                           && candidate.batt2_percent <= 100;
                };

                const auto little_endian = decode_detector_info(false);
                const auto big_endian = decode_detector_info(true);
                if (plausible_detector_info(little_endian)) {
                    info = little_endian;
                } else if (plausible_detector_info(big_endian)) {
                    info = big_endian;
                } else {
                    info = little_endian;
                }
            }

            std::cout << "Detector Info: serial=" << info.serial_number
                      << ", temp=" << info.detector_temp_kelvin << " K"
                      << ", biasStatus=" << info.detector_bias_status
                      << ", linePower=" << info.line_power_status
                      << ", batt1=" << info.batt1_percent << "%"
                      << ", batt2=" << info.batt2_percent << "%" << std::endl;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to get detector info: " << ex.what() << std::endl;
        }

        return info;
    }

    TcpEventReader::~TcpEventReader() {
        stopListening();
        disconnect();
    }

} // namespace phds_gegi_driver::socket_comms
