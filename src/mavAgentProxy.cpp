#include <chrono>
#include <future>
#include <memory>
#include <queue>
#include <vector>
#include <string>
#include <thread>

#include <sockpp/udp_socket.h>

#include <fmt/core.h>
#include <fmt/color.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "utils.h"
#include <mavlink.h>

#define maplog "[mavAgentProxy]"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx

// std::vector<uint8_t> agent_id = {1, 2, 3};

struct usocket
{
    std::shared_ptr<sockpp::udp_socket> sock;
    sockpp::inet_address gcs;
    sockpp::inet_address all;
};

std::vector<usocket> sockets;

int main()
{
    YAML::Node parameters = YAML::LoadFile("config/sim_urls.yaml");

    std::vector<std::string> urls = 
        parameters["mav_urls"].as<std::vector<std::string>>();

    if (urls.empty()) 
    {
        fmt::print(fg(fmt::color::crimson), "{} no uavs found in yaml\n", maplog);
        return 1;
    }

    sockpp::initialize();

    
    for (auto &url : urls)
    {
        std::vector<std::string> string_list = 
            utils::split_colon_delimiter(url);
        
        std::string ip;
        // https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html
        // string_list[0] = connection
        // string_list[1] = host/path
        // string_list[2] = port/baudrate
        if (std::strcmp(string_list[0].c_str(), "udp") == 0)
        {
            ip = string_list[1];
            ip.erase(0,2);
            usocket s;
            s.gcs = sockpp::inet_address(
                ip, std::stoi(string_list[2]) + 100);
            s.all = sockpp::inet_address(std::stoi(string_list[2]));
            s.sock =
                std::make_shared<sockpp::udp_socket>(sockpp::udp_socket());
            if (!s.sock->bind(s.all)) 
            {
                fmt::print(fg(fmt::color::crimson), 
                    "{} error connecting to all_sock at {}:{} {}\n", maplog,
                    "INADDR_ANY", string_list[2], s.sock->last_error_str());
                return 1;
            }
            sockets.emplace_back(s);
        }
        else 
            continue;
        
        fmt::print(fg(fmt::color::light_green), 
            "{} created UDP socket for {}\n", maplog, url);
    } 

    
    while(1)
    {
        uint8_t agent_id = 1;
        for (auto &sock : sockets)
        {
            uint8_t buf[BUFFER_LENGTH];
            mavlink_message_t msg;
            uint16_t len;

            // send heartbeat
            mavlink_msg_heartbeat_pack(
                agent_id, 
                1, 
                &msg, 
                MAV_TYPE_QUADROTOR, 
                MAV_AUTOPILOT_GENERIC, 
                MAV_MODE_MANUAL_DISARMED, 
                0, 
                MAV_STATE_STANDBY);
            uint16_t hb_len = mavlink_msg_to_send_buffer(buf, &msg);
            ssize_t hb_bytes_sent = sock.sock->send_to((void*)buf, hb_len, sock.gcs);

            // send status
            mavlink_msg_sys_status_pack(
                agent_id,
                1, 
                &msg, 
                0, 0, 0, 500, 11000, 
                -1, -1, 0, 0, 0, 
                0, 0, 0);
            uint16_t status_len = mavlink_msg_to_send_buffer(buf, &msg);
            ssize_t status_bytes_sent = sock.sock->send_to((void*)buf, status_len, sock.gcs);
            fmt::print(fg(fmt::color::sky_blue), "{} [{}] sent heartbeat and status! \n", 
                maplog, agent_id);

            memset(buf, 0, BUFFER_LENGTH);
            ssize_t receiver = sock.sock->recv_from(
                (void*)buf, BUFFER_LENGTH, &(sock.gcs));
            
            agent_id++;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}