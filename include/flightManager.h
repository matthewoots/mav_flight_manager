#ifndef FLIGHTMANAGER_H
#define FLIGHTMANAGER_H

#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include <string>
#include <mutex>

#include <fmt/core.h>
#include <fmt/color.h>
#include <yaml-cpp/yaml.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/PoseStampedPublisher.h"

#include <Eigen/Dense>

#define fmlog "[flightManager]"
#define poseGroup "pose"
#define trajectoryGroup "trajectory"

typedef std::chrono::time_point<std::chrono::system_clock> t_p_sc; // giving a typename

namespace flightManager
{
    // 3 threads per manager
    // 1. Mavlink Subscriber
    // 2. Mavlink Publisher
    // 3. DDS Subscriber

    enum flight_states
    {
        GROUND, // on ground
        TAKEOFF, // takeoff sequence
        MISSION_INTERAL, // move according to internal command
        MISSION_EXTERNAL, // move according to external command
        HOVER, // stop and hover
        LAND, // landing sequence
        EMERGENCY // emergency
    };   

    // Table of Contents specifically for the uav
    struct toc
    {
        t_p_sc t;
        uint8_t sys_id;
        uint8_t flight_state;
        bool is_local_position_ready;
        bool is_offboard;
        bool connected;
        Eigen::Affine3f nwuTransform;
    };

    std::map<uint8_t, toc> _agents_map;
    std::mutex _mutex;

    std::string flight_state_printer(uint8_t s)
    {
        switch(s)
        {
            case flight_states::GROUND:
                return "GROUND";
            case flight_states::TAKEOFF:
                return "TAKEOFF";
            case flight_states::MISSION_INTERAL:
                return "MISSION_INTERAL";
            case flight_states::MISSION_EXTERNAL:
                return "MISSION_EXTERNAL";
            case flight_states::HOVER:
                return "HOVER";
            case flight_states::LAND:
                return "LAND";
            case flight_states::EMERGENCY:
                return "EMERGENCY";
            default:
                return "ERROR";
        }
    }

    void agent_status_publisher(
        std::shared_ptr<mavsdk::System> _system);

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
                    context<fsm>()._iterator->second.flight_state = 
                        flightManager::TAKEOFF;
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

                fmt::print(fg(fmt::color::light_green), 
                    "{} mav{} enter FlightManager setup\n", 
                    fmlog, _mavSystem->get_system_id());
                
                _fsm = std::make_shared<fsm>();

                toc _tableOfContents;
                _tableOfContents.t = std::chrono::system_clock::now();
                _tableOfContents.flight_state = flightManager::GROUND;
                _tableOfContents.sys_id = _mavSystem->get_system_id();

                _agents_map.insert(std::pair
                    <uint8_t, toc>(_tableOfContents.sys_id, 
                    _tableOfContents));
                
                _iterator = 
                    _agents_map.find(_mavSystem->get_system_id());

                _offboard = 
                    std::make_shared<mavsdk::Offboard>(_mavSystem);

                // create a thread for subscription
                std::thread t(
                    &flightManager::agent_status_publisher,
                    _mavSystem);
                threads.push_back(std::move(t));

                // create a thread for publishing

                _fsm->setup(_tableOfContents.sys_id);
                _fsm->initiate();
                _fsm->process_event(EvChangeState());
                _fsm->process_event(EvTakeoff());
            };

            void fm_print_toc()
            {
                float x = std::round(_iterator->second.nwuTransform.translation().x() * 100.0) / 100.0;
                float y = std::round(_iterator->second.nwuTransform.translation().y() * 100.0) / 100.0;
                float z = std::round(_iterator->second.nwuTransform.translation().z() * 100.0) / 100.0;

                fmt::print( 
                    "{} mav{} {} pos({} {} {})\n", 
                    fmlog, _iterator->second.sys_id, 
                    flight_state_printer(_iterator->second.flight_state), 
                    x, y, z);
            };

            ~FlightManager(){};

    };
}

#endif