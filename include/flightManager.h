#ifndef FLIGHTMANAGER_H
#define FLIGHTMANAGER_H

#include <mutex>

#include <fmt/core.h>
#include <fmt/color.h>
#include <yaml-cpp/yaml.h>

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/PoseStampedPublisher.h"

#include "custom_msgs/msg/AgentCommand.h"
#include "custom_msgs/msg/AgentCommandSubscriber.h"
#include "custom_msgs/msg/AgentStatus.h"
#include "custom_msgs/msg/AgentStatusPublisher.h"

#include "utils.h"

#define fmlog "[flightManager]"
#define poseGroup "pose"
#define trajectoryGroup "trajectory"

using namespace utils;

namespace flightManager
{
    // 3 threads per manager
    // 1. Mavlink Subscriber
    // 2. Mavlink Publisher
    // 3. DDS Subscriber

    std::map<uint8_t, toc> _agents_map;
    std::mutex _mutex;

    void mav_dds_bridge_handler(
        std::shared_ptr<mavsdk::System> _system, double status_pub_rate);
    
    void user_command_handler();

    // void trigger_publisher();
    
    class FlightManager
    {
        private:

            struct ground;
            struct takeoff;
            struct mission_internal;
            struct hover;
            struct land;
            struct emergency;

            struct fsm : boost::statechart::state_machine<fsm, ground>
            {
                public:
                    std::map<uint8_t, toc>::iterator _iterator;

                    void setup(uint8_t sys_id) 
                    {
                        _iterator = _agents_map.find(sys_id);
                    }
            };
           
            std::shared_ptr<mavsdk::System> _mavSystem;
            std::shared_ptr<mavsdk::Offboard> _offboard;
            std::shared_ptr<fsm> _fsm;
            std::map<uint8_t, toc>::iterator _iterator;
            
            std::vector<std::thread> threads;
            double takeoff_height;
            double takeoff_land_velocity;
            double offboard_command_rate;
            double status_rate;

            struct EvChangeState : boost::statechart::event<EvChangeState> {};
            struct EvTakeoff : boost::statechart::event<EvTakeoff> {};
            struct EvHover : boost::statechart::event<EvHover> {};

            struct ground : boost::statechart::simple_state<ground, fsm>
            {
                typedef boost::statechart::transition<
                    EvChangeState, takeoff> reactions;
                
                ~ground()
                {
                    fmt::print(fg(fmt::color::light_green), 
                        "{} mav{} GROUND -> TAKEOFF\n", 
                        "[fsm]", context<fsm>()._iterator->first);
                    context<fsm>()._iterator->second.flight_state = TAKEOFF;
                }
            };

            struct takeoff : boost::statechart::simple_state<takeoff, fsm>
            {                
                typedef boost::statechart::custom_reaction<EvTakeoff> reactions;
                boost::statechart::result react(const EvTakeoff &);
            };

            struct hover : boost::statechart::simple_state<hover, fsm>
            {                
                
            };

            void user_command_subscriber()
            {
                // if takeoff
                _fsm->process_event(EvChangeState());
            }

        public:

            FlightManager(
                std::shared_ptr<mavsdk::System> mavSystem)
            {
                _mavSystem = std::move(mavSystem);

                YAML::Node parameters = 
                    YAML::LoadFile("config/common_parameters.yaml");
                
                takeoff_height = 
                    parameters["general"]["takeoff_height"].as<double>();
                takeoff_land_velocity = 
                    parameters["general"]["takeoff_land_velocity"].as<double>();
                offboard_command_rate = 
                    parameters["general"]["offboard_command_rate"].as<double>();
                status_rate = 
                    parameters["general"]["status_rate"].as<double>();

                fmt::print(fg(fmt::color::light_green), 
                    "{} mav{} enter FlightManager setup\n", 
                    fmlog, _mavSystem->get_system_id());
                
                _fsm = std::make_shared<fsm>();

                toc _tableOfContents;
                _tableOfContents.t = std::chrono::system_clock::now();
                _tableOfContents.flight_state = GROUND;
                _tableOfContents.sys_id = _mavSystem->get_system_id();

                _agents_map.insert(std::pair
                    <uint8_t, toc>(_tableOfContents.sys_id, 
                    _tableOfContents));
                
                _iterator = 
                    _agents_map.find(_mavSystem->get_system_id());

                _offboard = 
                    std::make_shared<mavsdk::Offboard>(_mavSystem);

                // create a thread for mavlink to dds
                std::thread t_m_d(
                    &flightManager::mav_dds_bridge_handler, 
                    _mavSystem, status_rate);
                threads.push_back(std::move(t_m_d));

                // create a thread for dds subscriber
                std::thread t_d_s(
                    &flightManager::user_command_handler);
                threads.push_back(std::move(t_d_s));

                _fsm->setup(_tableOfContents.sys_id);
                _fsm->initiate();
                // _fsm->process_event(EvChangeState());
                // _fsm->process_event(EvTakeoff());
            };

            void fm_print_toc()
            {
                float x = std::round(_iterator->second.nwuTransform.translation().x() * 100.0) / 100.0;
                float y = std::round(_iterator->second.nwuTransform.translation().y() * 100.0) / 100.0;
                float z = std::round(_iterator->second.nwuTransform.translation().z() * 100.0) / 100.0;

                fmt::print( 
                    "{} mav{} {} pos({} {} {})\n", 
                    fmlog, _iterator->second.sys_id, 
                    get_flight_state(_iterator->second.flight_state), 
                    x, y, z);
            };

            ~FlightManager(){};
    };
}

#endif