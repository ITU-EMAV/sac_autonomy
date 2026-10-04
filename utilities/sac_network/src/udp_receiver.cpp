#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <string>
#include <thread>
#include <cerrno>
#include <array>
#include <cmath>
#include <optional>
#include <vector>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using std::placeholders::_1;

class WheelFeedbackReceiverNode : public rclcpp::Node
{
public:
    WheelFeedbackReceiverNode() : Node("wheel_feedback_udp_receiver")
    {
        // Declare and get the UDP port parameter
        this->declare_parameter<int>("udp_port", 4950);
        int port;
        this->get_parameter("udp_port", port);

        // Create a UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to create UDP socket");
            return;
        }
        timeval receive_timeout{0, 20000};
        setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO,
                   &receive_timeout, sizeof(receive_timeout));

        // Bind the socket to the specified port
        sockaddr_in server_address{};
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(port);
        server_address.sin_addr.s_addr = INADDR_ANY;

        if (bind(udp_socket_, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind UDP socket to port %d", port);
            close(udp_socket_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Listening for UDP packets on port: %d", port);

        // Create a publisher
        encoder_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("encoder_speed", 1);
        steering_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("steering_angle", 1);

        // Start a thread for receiving UDP data
        udp_thread_ = std::thread([this]()
                                  { this->receiveUdp(); });
    }

    ~WheelFeedbackReceiverNode()
    {
        // Unblock recvfrom before joining; otherwise launch shutdown can hang
        // indefinitely when the gateway is silent.
        if (udp_socket_ >= 0)
        {
            shutdown(udp_socket_, SHUT_RDWR);
            close(udp_socket_);
            udp_socket_ = -1;
        }
        if (udp_thread_.joinable())
        {
            udp_thread_.join();
        }
    }

private:
    int udp_socket_;
    std::thread udp_thread_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr encoder_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_pub_;

    void receiveUdp()
    {
        char buffer[1024];
        sockaddr_in client_address{};
        socklen_t client_address_len = sizeof(client_address);

        while (rclcpp::ok())
        {
            // Receive UDP data
            int bytes_received = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                                          (struct sockaddr *)&client_address, &client_address_len);

            if (bytes_received < 0)
            {
                if (!rclcpp::ok())
                {
                    break;
                }
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    RCLCPP_ERROR(this->get_logger(), "Wheel UDP recvfrom failed: %s", strerror(errno));
                }
                continue;
            }
            if (bytes_received == 0)
            {
                if (!rclcpp::ok()) break;
                continue;
            }

            buffer[bytes_received] = '\0'; // Null-terminate the received data
            std::string message(buffer);

            try
            {
                // Parse the received message as JSON
                auto json_message = json::parse(message);

                // Extract the encoder_speed field
                if (json_message.contains("encoder_speed"))
                {
                    float encoder_speed = json_message.at("encoder_speed").get<float>();

                    // Publish the encoder_speed as a Float32 message
                    auto msg = std_msgs::msg::Float32();
                    msg.data = encoder_speed;

                    // RCLCPP_INFO(this->get_logger(), "Received encoder_speed: %f", encoder_speed);
                    encoder_speed_pub_->publish(msg);

                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "JSON does not contain 'encoder_speed' field");
                }

                // Extract steering angle
                if (json_message.contains("steering_angle_in_deg"))
                {
                    float steering_angle_in_deg = json_message.at("steering_angle_in_deg").get<float>();

                    // Publish the encoder_speed as a Float32 message
                    auto msg = std_msgs::msg::Float32();
                    msg.data = steering_angle_in_deg;

                    // RCLCPP_INFO(this->get_logger(), "Received steering_angle_in_deg: %f", steering_angle_in_deg);
                    steering_angle_pub_->publish(msg);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "JSON does not contain 'encoder_speed' field");
                }
            }
            catch (const json::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
            }
        }
    }
};

class GPSUDPNode : public rclcpp::Node
{
public:
    int udp_socket_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    GPSUDPNode() : Node("gps_udp_node")
    {
        const int port = this->declare_parameter<int>("gps_udp_port", 5017);
        // Create a publisher for the NavSatFix message
        publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/navsat/fix", 1);
        // Create a publisher
        velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("/navsat/velocity", 1);

        // Initialize UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            return;
        }
        timeval receive_timeout{0, 20000};
        setsockopt(udp_socket_, SOL_SOCKET, SO_RCVTIMEO,
                   &receive_timeout, sizeof(receive_timeout));

        // Set up UDP socket
        sockaddr_in server_address{};
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(port);
        server_address.sin_addr.s_addr = INADDR_ANY;

        if (bind(udp_socket_, (struct sockaddr *)&server_address, sizeof(server_address)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket to port %d", port);
            close(udp_socket_);
            return;
        }

        // Start a timer to read data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&GPSUDPNode::receive_and_publish_data, this));

        RCLCPP_INFO(this->get_logger(), "GPS UDP Node started, listening on port %d", port);
    }

    ~GPSUDPNode()
    {
        close(udp_socket_);
    }

private:
    std::optional<std::array<double, 3>> latest_gst_variance_;
    std::chrono::steady_clock::time_point latest_gst_time_;

    static std::vector<std::string> fields(const std::string &sentence)
    {
        std::vector<std::string> result;
        std::istringstream stream(sentence.substr(0, sentence.find('*')));
        std::string field;
        while (std::getline(stream, field, ',')) result.push_back(field);
        return result;
    }

    static std::optional<double> positive_number(const std::string &value)
    {
        try
        {
            size_t used = 0;
            const double number = std::stod(value, &used);
            if (used == value.size() && std::isfinite(number) && number > 0.0) return number;
        }
        catch (const std::exception &) {}
        return std::nullopt;
    }

    void receive_and_publish_data()
    {
        char buffer[1024] = {0};
        sockaddr_in client_address{};
        socklen_t client_address_len = sizeof(client_address);

        // Receive data from the UDP socket
        int bytes_received = recvfrom(udp_socket_, buffer, sizeof(buffer) - 1, 0,
                                      (struct sockaddr *)&client_address, &client_address_len);

        if (bytes_received < 0)
        {
            if (errno != EAGAIN && errno != EWOULDBLOCK && rclcpp::ok())
            {
                RCLCPP_WARN(this->get_logger(), "GPS recvfrom failed: %s", strerror(errno));
            }
            return;
        }

        buffer[bytes_received] = '\0';
        std::string message(buffer, bytes_received);
        std::vector<std::string> sentences;
        std::istringstream lines(message);
        std::string line;
        while (std::getline(lines, line))
        {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) sentences.push_back(line);
        }

        std::optional<std::array<double, 3>> gst_variance;
        for (const auto &sentence : sentences)
        {
            const auto data = fields(sentence);
            if (data.empty()) continue;
            if (data[0].size() == 6 && data[0].substr(3) == "GST" && data.size() > 8)
            {
                const auto north = positive_number(data[6]);
                const auto east = positive_number(data[7]);
                const auto up = positive_number(data[8]);
                if (north && east && up)
                {
                    gst_variance = std::array<double, 3>{*east * *east, *north * *north, *up * *up};
                    latest_gst_variance_ = gst_variance;
                    latest_gst_time_ = std::chrono::steady_clock::now();
                }
            }
        }
        for (const auto &sentence : sentences)
        {
            const auto data = fields(sentence);
            if (data.empty() || data[0].size() != 6) continue;
            if (data[0].substr(3) == "GGA")
            {
                try
                {
                    auto fix = parse_nmea_gga(sentence);
                    if (!fix) continue;
                    const bool recent_gst = latest_gst_variance_ &&
                        std::chrono::steady_clock::now() - latest_gst_time_ < std::chrono::seconds(1);
                    const auto variance = gst_variance ? gst_variance :
                        (recent_gst ? latest_gst_variance_ : std::nullopt);
                    if (variance)
                    {
                        fix->position_covariance[0] = (*variance)[0];
                        fix->position_covariance[4] = (*variance)[1];
                        fix->position_covariance[8] = (*variance)[2];
                        fix->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
                    }
                    // Without receiver-provided GST, leave covariance UNKNOWN.
                    publisher_->publish(*fix);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Invalid NMEA-GGA message: %s", e.what());
                }
            }
            else if (data[0].substr(3) == "RMC")
            {
                try
                {
                    auto velocity = parse_nmea_rmc(sentence);
                    if (velocity) velocity_pub_->publish(*velocity);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Invalid NMEA-RMC message: %s", e.what());
                }
            }
        }
    }

    std::optional<sensor_msgs::msg::NavSatFix> parse_nmea_gga(const std::string &message)
    {
        std::istringstream stream(message);
        std::string type, time, lat, lat_dir, lon, lon_dir, fix_quality, num_satellites, hdop, altitude;

        std::getline(stream, type, ',');
        std::getline(stream, time, ',');
        std::getline(stream, lat, ',');
        std::getline(stream, lat_dir, ',');
        std::getline(stream, lon, ',');
        std::getline(stream, lon_dir, ',');
        std::getline(stream, fix_quality, ',');
        std::getline(stream, num_satellites, ','); // This is the number of satellites
        std::getline(stream, hdop, ',');
        std::getline(stream, altitude, ',');

        if (lat.empty() || lon.empty() || altitude.empty() || fix_quality.empty() || num_satellites.empty())
        {
            return std::nullopt;
        }

        // Convert latitude and longitude to decimal degrees
        double latitude = std::stod(lat.substr(0, 2)) + std::stod(lat.substr(2)) / 60.0;
        double longitude = std::stod(lon.substr(0, 3)) + std::stod(lon.substr(3)) / 60.0;

        // Adjust direction
        if (lat_dir == "S")
            latitude = -latitude;
        if (lon_dir == "W")
            longitude = -longitude;

        // Create and populate NavSatFix message
        sensor_msgs::msg::NavSatFix navsat_msg;
        navsat_msg.header.stamp = this->get_clock()->now();
        navsat_msg.header.frame_id = "gps";

        if (fix_quality == "0")
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000, // 5 saniyede bir
                "GPS has no fix.");
            return std::nullopt;
        }

        navsat_msg.status.status = fix_quality == "0" ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

        navsat_msg.latitude = latitude;
        navsat_msg.longitude = longitude;
        navsat_msg.altitude = std::stod(altitude);

        // Print the number of satellites
        // int satellites = std::stoi(num_satellites);
        // RCLCPP_INFO(this->get_logger(), "Number of satellites in view: %d", satellites);
        // RCLCPP_INFO(this->get_logger(), "HDOP: %f", stod(hdop));

        return navsat_msg;
    }

    std::optional<std_msgs::msg::Float32> parse_nmea_rmc(const std::string &message)
    {
        std::istringstream stream(message);
        std::string type, time, status, lat, lat_dir, lon, lon_dir, speed, track_angle;

        std::getline(stream, type, ',');
        std::getline(stream, time, ',');
        std::getline(stream, status, ',');
        std::getline(stream, lat, ',');
        std::getline(stream, lat_dir, ',');
        std::getline(stream, lon, ',');
        std::getline(stream, lon_dir, ',');
        std::getline(stream, speed, ',');
        std::getline(stream, track_angle, ',');
        if (status != "A" || speed.empty())
        {
            return std::nullopt;
        }
        auto msg = std_msgs::msg::Float32();
        msg.data = std::stof(speed) * 0.514444f; // NMEA RMC speed is knots.

        return msg;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create shared pointers to the nodes
    auto gps_node = std::make_shared<GPSUDPNode>();
    auto wheel_odom_node = std::make_shared<WheelFeedbackReceiverNode>();

    // Use a multithreaded executor to run both nodes
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(gps_node);
    executor.add_node(wheel_odom_node);

    // Spin the executor
    executor.spin();

    // Shutdown after spinning
    rclcpp::shutdown();
    return 0;
}
