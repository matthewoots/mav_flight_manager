#include <chrono>
#include <future>
#include <memory>
#include <queue>
#include <vector>
#include <string>
#include <thread>
#include <random>

#include <sockpp/udp_socket.h>

#include <fmt/core.h>
#include <fmt/color.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "utils.h"
#include <mavlink.h>

#define maplog "[mavAgentProxy]"

#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx

void help(const std::string& bin_name)
{
    fmt::print(fg(fmt::color::orange_red),
        "{} {} --help\n", maplog, bin_name);
    fmt::print(fg(fmt::color::orange),
        "{} format : {} <type-of-conenction> <number-of-drones>\n", maplog, bin_name);
    fmt::print(
        "{} <type-of-conenction> should be either 'single' or 'multi' or 'hybrid'\n", maplog);
    fmt::print(
        "{}     for single : all mavlink messages will stream through a single entry point\n", maplog);
    fmt::print(
        "{}     for multi : each will have their own entry point (port)\n", maplog);
    fmt::print(
        "{}     for hybrid : <number-of-drones> = <number of drones> <number-of-ports>\n", maplog);
}

struct usocket
{
    std::shared_ptr<sockpp::udp_socket> sock;
    sockpp::inet_address gcs;
    sockpp::inet_address all;
};

struct agent_network
{
    std::vector<std::pair<uint8_t, Eigen::Affine3f>> agents;
    uint64_t port;
    usocket usock;
    std::string url;
};

enum conn_method
{
    SINGLE,
    MULTI,
    HYBRID
};

Eigen::Vector3f get_random_point_in_circle(float radius)
{
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_real_distribution<float> distribution(0, 1);
    float r = radius * std::sqrt(distribution(rng));
    float theta = distribution(rng) * 2 * M_PI;
    
    return Eigen::Vector3f(
        r * std::cos(theta),
        r * std::sin(theta),
        0.0
    );
}

int main(int argc, char* argv[])
{
    uint8_t conn;
    // do checks for the executable and arguments
    // if not single or multi
    if (!std::strcmp(argv[1], "single"))
        conn = SINGLE;
    else if (!std::strcmp(argv[1], "multi"))
        conn = MULTI;
    else if (std::strcmp(argv[1], "hybrid"))
        conn = HYBRID;
    else
    {
        help(argv[0]);
        exit(EXIT_FAILURE); 
    }
    
    if ((conn == HYBRID && argc != 4) ||
        ((conn == SINGLE || conn == MULTI) && argc != 3)) 
    {
        help(argv[0]);
        exit(EXIT_FAILURE);
    }
    
    std::vector<agent_network> nets;

    std::string base_udp_ip = "127.0.0.1"; 
    uint64_t port = 14580;
    uint64_t number_of_agents = std::stoi(argv[2]);

    float radius = 6.0;

    sockpp::initialize();

    switch (conn)
    {
        case SINGLE:
        {
            agent_network a_n;
            a_n.port = port;
            for (uint64_t i = 0; i < number_of_agents; i++)
            {
                Eigen::Affine3f transform;
                transform.translation() = get_random_point_in_circle(radius);
                a_n.agents.emplace_back(
                    std::pair<uint8_t, Eigen::Affine3f>(i+1, transform));
                fmt::print(fg(fmt::color::light_green), 
                    "{} id{} position {} {} {}\n", 
                    maplog, i+1, transform.translation().x(), transform.translation().y(), transform.translation().z());
            }
            a_n.usock.gcs = sockpp::inet_address(
                base_udp_ip, a_n.port + 100);
            a_n.usock.all = sockpp::inet_address(a_n.port);
            a_n.usock.sock =
                std::make_shared<sockpp::udp_socket>(sockpp::udp_socket());
            if (!a_n.usock.sock->bind(a_n.usock.all)) 
            {
                fmt::print(fg(fmt::color::crimson), 
                    "{} error connecting to all_sock at {}:{} {}\n", maplog,
                    "INADDR_ANY", a_n.port, a_n.usock.sock->last_error_str());
                return 1;
            }
            nets.emplace_back(a_n);

            fmt::print(fg(fmt::color::light_green), 
                "{} created UDP socket for {}:{}\n", 
                maplog, base_udp_ip, a_n.port);
            break;
        }
        case MULTI:
        {
            for (uint64_t i = 0; i < number_of_agents; i++)
            {
                agent_network a_n;
                a_n.port = port+i;
                a_n.usock.gcs = sockpp::inet_address(
                    base_udp_ip, a_n.port + 100);
                a_n.usock.all = sockpp::inet_address(a_n.port);
                a_n.usock.sock =
                    std::make_shared<sockpp::udp_socket>(sockpp::udp_socket());
                if (!a_n.usock.sock->bind(a_n.usock.all)) 
                {
                    fmt::print(fg(fmt::color::crimson), 
                        "{} error connecting to all_sock at {}:{} {}\n", maplog,
                        "INADDR_ANY", a_n.port, a_n.usock.sock->last_error_str());
                    return 1;
                }
                Eigen::Affine3f transform;
                transform.translation() = get_random_point_in_circle(radius);
                fmt::print(fg(fmt::color::light_green), 
                    "{} id{} position {} {} {}\n", 
                    maplog, i+1, transform.translation().x(), transform.translation().y(), transform.translation().z());
                a_n.agents.emplace_back(
                    std::pair<uint8_t, Eigen::Affine3f>(i+1, transform));
                nets.emplace_back(a_n);

                fmt::print(fg(fmt::color::light_green), 
                    "{} created UDP socket for {}:{}\n", 
                    maplog, base_udp_ip, a_n.port);
            }
            break;
        }
        default:
            exit(EXIT_FAILURE);
    }
    
    while(1)
    {
        for (auto &net : nets)
        {
            uint8_t buf[BUFFER_LENGTH];
            mavlink_message_t msg;
            uint16_t len;
            
            for (const auto& agent : net.agents)
            {
                // mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                //      uint8_t type, uint8_t autopilot, uint8_t base_mode, 
                //      uint32_t custom_mode, uint8_t system_status)
                // send heartbeat
                mavlink_msg_heartbeat_pack(
                    agent.first, 200, &msg, 
                    MAV_TYPE_QUADROTOR, 
                    MAV_AUTOPILOT_PX4, 
                    MAV_MODE_MANUAL_DISARMED, 
                    0, 
                    MAV_STATE_STANDBY);
                uint16_t hb_len = 
                    mavlink_msg_to_send_buffer(buf, &msg);
                ssize_t hb_bytes_sent = 
                    net.usock.sock->send_to((void*)buf, hb_len, net.usock.gcs);

                // w, x, y, z
                Eigen::Quaternionf qe(agent.second.linear());
                const float q[4] = {qe.w(), qe.x(), qe.y(), qe.z()};

                const float cov[21] = {0};

                // mavlink_msg_odometry_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                //      uint64_t time_usec, uint8_t frame_id, uint8_t child_frame_id, 
                //      float x, float y, float z, const float *q, float vx, float vy, 
                //      float vz, float rollspeed, float pitchspeed, float yawspeed, const float *pose_covariance, const float *velocity_covariance, uint8_t reset_counter)
                mavlink_msg_odometry_pack(
                    agent.first, 200, &msg,
                    0, 1, 0,
                    agent.second.translation().x(), agent.second.translation().y(), agent.second.translation().z(),
                    q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov, cov, 0);
                uint16_t odom_len = 
                    mavlink_msg_to_send_buffer(buf, &msg);
                ssize_t odom_bytes_sent = 
                    net.usock.sock->send_to((void*)buf, odom_len, net.usock.gcs);
                
                fmt::print(fg(fmt::color::sky_blue), "{} [{}] sent heartbeat and odom! \n", 
                    maplog, agent.first);
            }

            memset(buf, 0, BUFFER_LENGTH);
            ssize_t receiver = net.usock.sock->recv_from(
                (void*)buf, BUFFER_LENGTH, &(net.usock.gcs));
        }

        fmt::print("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return 0;
}