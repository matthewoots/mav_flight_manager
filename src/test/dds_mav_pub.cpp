#include <chrono>
#include <vector>
#include <string>
#include <mutex>

#include <fmt/core.h>
#include <fmt/color.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include <Eigen/Dense>
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/PoseStampedPublisher.h"

#define mlog "[main]"

mavsdk::Mavsdk mav_sdk;
using namespace std::literals::chrono_literals;

std::mutex agent_status_mutex;

struct agent_status
{
    int8_t sys_id;
    bool is_local_position_ready;
    bool is_offboard;
    Eigen::Affine3f nwuTransform;
};

std::map<int8_t, agent_status> a_m;

static std::string get_current_time_string()
{
    time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return s;
}

void agent_status_publisher(
    std::shared_ptr<mavsdk::System> system)
{
    PoseStampedPublisher pose_stamped_pub;

    std::map<int8_t, agent_status>::iterator iter = a_m.find(system->get_system_id());
    iter->second.sys_id = system->get_system_id();
    fmt::print("{} discovered system id {}\n", mlog, iter->second.sys_id);
    pose_stamped_pub.init("rt/mav" + std::to_string(iter->second.sys_id) + "/pose");

    auto telemetry = mavsdk::Telemetry{system};
    telemetry.subscribe_odometry([&](mavsdk::Telemetry::Odometry odom) 
    {
        mavsdk::Telemetry::Odometry::MavFrame frame = odom.frame_id;
        if (odom.frame_id == mavsdk::Telemetry::Odometry::MavFrame::BodyNed)
        {
            // convert from ned to nwu
            // https://github.com/FroboLab/frobomind/blob/master/fmSensors/global_sensing/imu/vectornav_vn100/src/vectornav_vn100_node.cpp
            auto pos_nwu = Eigen::Vector3f(
                odom.position_body.x_m, 
                -odom.position_body.y_m, 
                -odom.position_body.z_m
            );

            agent_status_mutex.lock();
            iter->second.nwuTransform.translation() = pos_nwu;

            // Quaternion format w, x, y, z
            Eigen::Quaternionf rot = Eigen::Quaternionf(
                odom.q.w, odom.q.x, 
                -odom.q.y, -odom.q.z
            );

            iter->second.nwuTransform.linear() = rot.toRotationMatrix();
            agent_status_mutex.unlock();

            // fmt::print("{} position {} {} {}\n", 
            //     mlog, iter->second.nwuTransform.translation().x(), iter->second.nwuTransform.translation().y(), iter->second.nwuTransform.translation().z());

            geometry_msgs::msg::PoseStamped pose_stamped_msg;

            auto current_time = std::chrono::system_clock::now();
            auto duration_in_seconds = std::chrono::duration<double>(current_time.time_since_epoch());
            double double_time = duration_in_seconds.count();

            double rounded_time = std::floor(double_time);
            int32_t sec = static_cast<int32_t>(rounded_time);
            int32_t nsec = static_cast<int32_t>((double_time - rounded_time) / std::pow(10,-9));

            pose_stamped_msg.header().stamp().sec() = sec;
            pose_stamped_msg.header().stamp().nanosec() = nsec;

            pose_stamped_msg.header().frame_id() = "nwu";

            pose_stamped_msg.pose().position().x() = pos_nwu.x();
            pose_stamped_msg.pose().position().y() = pos_nwu.y();
            pose_stamped_msg.pose().position().z() = pos_nwu.z();

            pose_stamped_msg.pose().orientation().w() = rot.w();
            pose_stamped_msg.pose().orientation().x() = rot.x();
            pose_stamped_msg.pose().orientation().y() = rot.y();
            pose_stamped_msg.pose().orientation().z() = rot.z();

            pose_stamped_pub.publish(&pose_stamped_msg);
        }        
    }
    );

    telemetry.subscribe_health([&](mavsdk::Telemetry::Health health) 
    {
        agent_status_mutex.lock();
        iter->second.is_local_position_ready = health.is_local_position_ok;
        agent_status_mutex.unlock();
    });

    // keep this thread running
    while (true)
        std::this_thread::sleep_for(1ms);
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        exit(EXIT_FAILURE);
    }

    double timeout = 2.0;

    // for simulation udp://127.0.0.1:14650
    mavsdk::ConnectionResult connection_result = 
        mav_sdk.add_any_connection(argv[1]);
    
    std::atomic<size_t> num_systems_discovered{0};

    fmt::print("{} waiting to discover system...\n", mlog);
    mav_sdk.subscribe_on_new_system(
        [&mav_sdk, &num_systems_discovered]() 
        {
            const auto systems = mav_sdk.systems();

            if (systems.size() > num_systems_discovered) {
                fmt::print(fg(fmt::color::light_green), "{} discovered system\n", mlog);
                num_systems_discovered = systems.size();
            }
        }
    );
    
    fmt::print("{} timeout in {}s...\n", mlog, timeout);
    // We usually receive heartbeats at 1Hz, therefore we should find a system after around <timeout> seconds.

    auto literal = timeout * 1000ms;
    std::this_thread::sleep_for(literal);

    fmt::print(fg(fmt::color::orange), "{} total discovered system {}\n", 
        mlog, mav_sdk.systems().size());

    if (mav_sdk.systems().size() == 0) {
        fmt::print(fg(fmt::color::crimson), "{} no systems found\n", mlog);
        exit(EXIT_FAILURE);
    }

    std::vector<std::thread> threads;
    std::vector<mavsdk::Offboard> offboards;
    for (auto system : mav_sdk.systems())
    {
        if (system->get_system_id() < 0)
            continue;
        a_m.insert(std::pair
            <int8_t, agent_status>(system->get_system_id(), agent_status()));
        
        std::thread t(&agent_status_publisher, system);
        // Instead of copying, move t into the vector (less expensive)
        threads.push_back(std::move(t));
        // offboards.push_back(mavsdk::Offboard{system});
        // offboards.back().set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
        // mavsdk::Offboard::Result offboard_result = offboards.back().start();
        // if (result != Offboard::Result::Success) 
        // {
        //     std::cerr << "Offboard::start() failed: " << offboard_result << '\n';
        // }
    }

    while (true)
    {
        for (const auto &agent : a_m) {
            agent_status_mutex.lock();
            if (!agent.second.is_local_position_ready)
            {
                fmt::print(fg(fmt::color::crimson), "{} no local position found\n", mlog);
                agent_status_mutex.unlock();
                std::this_thread::sleep_for(1000ms);
                continue;
            }
            agent_status_mutex.unlock();
        }

        std::this_thread::sleep_for(100ms);
    }

    for (auto& t : threads) 
    {
        t.join();
    }

    
    exit(EXIT_SUCCESS);
}

