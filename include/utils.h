/*
* utils.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2023 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <vector>
#include <regex>
#include <string>
#include <chrono>

#include <boost/assign.hpp>

#include <Eigen/Dense>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

namespace utils
{
    typedef std::chrono::time_point<std::chrono::system_clock> t_p_sc; // giving a typename

    enum flight_states
    {
        GROUND, // on ground
        TAKEOFF, // takeoff sequence
        MISSION_INTERAL, // move according to internal command
        MISSION_EXTERNAL, // move according to external command
        HOVER, // stop and hover
        LAND, // landing sequence
        EMERGENCY, // emergency
        ERROR // error in flight state
    };   

    // Table of Contents specifically for the uav
    struct toc
    {
        t_p_sc t;
        uint8_t sys_id;
        uint8_t flight_state;
        mavsdk::Telemetry::FlightMode flight_mode;
        bool is_local_position_ready;
        bool is_offboard;
        bool connected;
        Eigen::Affine3f nwuTransform;
    };

    struct custom_commands
    {
        enum CMD
        {
            CTAKEOFF,
            CHOLD,
            CMOVE,
            CEXTERNAL,
            CLAND
        };
        enum SYNC
        {
            SYNC_START_END,
            SYNC_START,
            SYNC_END,
            NOSYNC
        };

        std::map<std::string, CMD> cmd_map = 
            boost::assign::map_list_of
            ("TAKEOFF", CMD::CTAKEOFF)
            ("HOLD", CMD::CHOLD)
            ("MOVE", CMD::CMOVE)
            ("EXTERNAL", CMD::CEXTERNAL)
            ("LAND", CMD::CLAND);
        
        std::map<std::string, SYNC> sync_map = 
            boost::assign::map_list_of
            ("SYNC_START_END", SYNC::SYNC_START_END)
            ("SYNC_START", SYNC::SYNC_START)
            ("SYNC_END", SYNC::SYNC_END)
            ("NOSYNC", SYNC::NOSYNC);
        
        int8_t string_to_cmd_uint(std::string str)
        {
            auto it = cmd_map.find(str);
            if (it != cmd_map.end())
                return it->second;
            else
                return -1;
        }

        int8_t string_to_sync_uint(std::string str)
        {
            auto it = sync_map.find(str);
            if (it != sync_map.end())
                return it->second;
            else
                return -1;
        }

        uint8_t command;
        uint8_t sync_mode;
        std::vector<int8_t> participants;
        double duration;
        std::vector<Eigen::Vector4f> target;
    };
    

    std::vector<std::string> split_space_delimiter(std::string str);
    
    std::vector<std::string> split_colon_delimiter(std::string str);

    std::vector<std::string> split_fullstop_delimiter(std::string str);

    std::string get_fc_flight_mode(
        mavsdk::Telemetry::FlightMode flight_mode);
    mavsdk::Telemetry::FlightMode get_fc_flight_mode(
        std::string flight_mode);

    std::string get_flight_state(uint8_t s);
    uint8_t get_flight_state(std::string s);
}

#endif