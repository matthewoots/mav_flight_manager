#include "flightManager.h"

static std::string get_current_time_string()
{
    time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string s(30, '\0');
    strftime(&s[0], s.size(), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return s;
}

void flightManager::agent_status_publisher(
    std::shared_ptr<mavsdk::System> _system)
{
    using namespace std::literals::chrono_literals;
    PoseStampedPublisher pose_stamped_pub;

    std::map<uint8_t, toc>::iterator iter = 
        _agents_map.find(_system->get_system_id());
    
    if (iter == _agents_map.end())
    {
        fmt::print(fg(fmt::color::crimson), 
            "{} cannot find {} ...\n", 
            fmlog, _system->get_system_id());
        return;
    }

    fmt::print("{} setup dds publisher on mav{} ...\n", 
        fmlog, iter->second.sys_id);
    pose_stamped_pub.init("rt/mav" + std::to_string(iter->second.sys_id) + "/pose");

    auto telemetry = mavsdk::Telemetry{_system};
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

            // Quaternion format w, x, y, z
            Eigen::Quaternionf rot = Eigen::Quaternionf(
                odom.q.w, odom.q.x, 
                -odom.q.y, -odom.q.z
            );

            _mutex.lock();
            iter->second.nwuTransform.translation() = pos_nwu;
            iter->second.nwuTransform.linear() = rot.toRotationMatrix();
            _mutex.unlock();

            // fmt::print("{} position {} {} {}\n", 
            //     fmlog, pos_nwu.x(), pos_nwu.y(), pos_nwu.z());

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
    });

    telemetry.subscribe_health([&](mavsdk::Telemetry::Health health) 
    {
        _mutex.lock();
        iter->second.is_local_position_ready = 
            health.is_local_position_ok;
        _mutex.unlock();
    });

    fmt::print("{} setup dds publisher on mav{} completed\n", 
        fmlog, iter->second.sys_id);

    // keep this thread running
    while (true)
        std::this_thread::sleep_for(5ms);
}

// void flightManager::trigger_publisher()
// {

// }

boost::statechart::result 
    flightManager::FlightManager::takeoff::react(const EvTakeoff &)
{
    using namespace std::literals::chrono_literals;

    // if ()
    //     flightManager::trigger_publisher();
    // return discard_event();

    fmt::print(fg(fmt::color::light_green), 
        "{} mav{} TAKEOFF -> HOVER\n", 
        "[fsm]", context<fsm>()._iterator->first);
    context<fsm>()._iterator->second.flight_state = 
        flightManager::HOVER;
    // while (1)
    // {
    //     Eigen::Vector3f pos = 
    //         context<fsm>()._toc->nwuTransform.translation();
    //     fmt::print(fg(fmt::color::light_green), 
    //         "{} mav{} pos({} {} {})\n", 
    //         "[fsm]", context<fsm>()._toc->sys_id,
    //         pos.x(), pos.y(), pos.z());
    //     std::this_thread::sleep_for(500ms);
    // }
    return transit<flightManager::FlightManager::hover>();
}


int main(int argc, char* argv[])
{
    using namespace std::literals::chrono_literals;
    mavsdk::Mavsdk mav_sdk;
    std::vector<std::shared_ptr<flightManager::FlightManager>> fms;

    if (argc < 2)
    {
        exit(EXIT_FAILURE);
    }

    // YAML::Node parameters = 
    //     YAML::LoadFile("config/common_parameters.yaml");
    
    // double takeoff_height = 
    //     parameters["takeoff_height"].as<double>();
    // double offboard_command_rate = 
    //     parameters["offboard_command_rate"].as<double>();

    double timeout = 2.0;

    // for simulation udp://127.0.0.1:14650
    mavsdk::ConnectionResult connection_result = 
        mav_sdk.add_any_connection(argv[1]);
    
    std::atomic<size_t> num_systems_discovered{0};

    fmt::print("{} waiting to discover system...\n", fmlog);
    mav_sdk.subscribe_on_new_system(
        [&mav_sdk, &num_systems_discovered]() 
        {
            const auto systems = mav_sdk.systems();

            if (systems.size() > num_systems_discovered) {
                fmt::print(fg(fmt::color::light_green), "{} discovered system\n", fmlog);
                num_systems_discovered = systems.size();
            }
        }
    );
    
    fmt::print("{} timeout in {}s...\n", fmlog, timeout);
    // We usually receive heartbeats at 1Hz, therefore we should find a system after around <timeout> seconds.

    auto literal = timeout * 1000ms;
    std::this_thread::sleep_for(literal);

    fmt::print(fg(fmt::color::orange), "{} total discovered system {}\n", 
        fmlog, mav_sdk.systems().size());

    if (mav_sdk.systems().size() == 0) {
        fmt::print(fg(fmt::color::crimson), "{} no systems found\n", fmlog);
        exit(EXIT_FAILURE);
    }
    
    for (auto system : mav_sdk.systems())
    {
        // remove any unwanted ids
        if (system->get_system_id() < 0 || system->get_system_id() == 255)
            continue;
        std::shared_ptr<flightManager::FlightManager> fm =
            std::make_shared<flightManager::FlightManager>(system);
        fms.emplace_back(fm);
    }

    while (true)
    {
        fmt::print(fg(fmt::color::orange), "{} {}\n",
            fmlog, get_current_time_string());
        for (auto fm : fms)
        {
            fm->fm_print_toc();
        }
        fmt::print("\n");
        std::this_thread::sleep_for(2000ms);
    }
    
    exit(EXIT_SUCCESS);
}