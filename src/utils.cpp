#include "utils.h"

std::vector<std::string> 
    utils::split_space_delimiter(std::string str)
{
    std::stringstream ss(str);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> vstrings(begin, end);
    return vstrings;
}

std::vector<std::string> 
    utils::split_colon_delimiter(std::string str)
{
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> output;
    while (std::getline(ss, item, ':'))
        output.push_back(item);
    
    return output;
}

std::vector<std::string> 
    utils::split_fullstop_delimiter(std::string str)
{
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> output;
    while (std::getline(ss, item, '.'))
        output.push_back(item);
    
    return output;
}

std::string utils::get_fc_flight_mode(
    mavsdk::Telemetry::FlightMode flight_mode)
{
    switch (flight_mode) 
    {
        case mavsdk::Telemetry::FlightMode::Unknown:
            return "Unknown";
        case mavsdk::Telemetry::FlightMode::Ready:
            return "Ready";
        case mavsdk::Telemetry::FlightMode::Takeoff:
            return "Takeoff";
        case mavsdk::Telemetry::FlightMode::Hold:
            return "Hold";
        case mavsdk::Telemetry::FlightMode::Mission:
            return "Mission";
        case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
            return "Return To Launch";
        case mavsdk::Telemetry::FlightMode::Land:
            return "Land";
        case mavsdk::Telemetry::FlightMode::Offboard:
            return "Offboard";
        case mavsdk::Telemetry::FlightMode::FollowMe:
            return "Follow Me";
        case mavsdk::Telemetry::FlightMode::Manual:
            return "Manual";
        case mavsdk::Telemetry::FlightMode::Altctl:
            return "Altctl";
        case mavsdk::Telemetry::FlightMode::Posctl:
            return "Posctl";
        case mavsdk::Telemetry::FlightMode::Acro:
            return "Acro";
        case mavsdk::Telemetry::FlightMode::Stabilized:
            return "Stabilized";
        case mavsdk::Telemetry::FlightMode::Rattitude:
            return "Rattitude";
        default:
            return "Unknown";
    }
}
mavsdk::Telemetry::FlightMode utils::get_fc_flight_mode(
    std::string flight_mode)
{
    if (std::strcmp("Unknown", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Unknown;
    else if (std::strcmp("Ready", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Ready;
    else if (std::strcmp("Takeoff", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Takeoff;
    else if (std::strcmp("Hold", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Hold;
    else if (std::strcmp("Mission", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Mission;
    else if (std::strcmp("Return To Launch", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::ReturnToLaunch;
    else if (std::strcmp("Land", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Land;
    else if (std::strcmp("Offboard", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Offboard;
    else if (std::strcmp("Follow Me", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::FollowMe;
    else if (std::strcmp("Manual", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Manual;
    else if (std::strcmp("Altctl", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Altctl;
    else if (std::strcmp("Posctl", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Posctl;
    else if (std::strcmp("Acro", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Acro;
    else if (std::strcmp("Stabilized", flight_mode.c_str()) == 0)
        return mavsdk::Telemetry::FlightMode::Stabilized;
    else
        return mavsdk::Telemetry::FlightMode::Unknown;
}

std::string utils::get_flight_state(uint8_t s)
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
uint8_t utils::get_flight_state(std::string s)
{
    if (std::strcmp("GROUND", s.c_str()) == 0)
        return flight_states::GROUND;
    else if (std::strcmp("TAKEOFF", s.c_str()) == 0)
        return flight_states::TAKEOFF;
    else if (std::strcmp("MISSION_INTERAL", s.c_str()) == 0)
        return flight_states::MISSION_INTERAL;
    else if (std::strcmp("MISSION_EXTERNAL", s.c_str()) == 0)
        return flight_states::MISSION_EXTERNAL;
    else if (std::strcmp("HOVER", s.c_str()) == 0)
        return flight_states::HOVER;
    else if (std::strcmp("LAND", s.c_str()) == 0)
        return flight_states::LAND;
    else if (std::strcmp("EMERGENCY", s.c_str()) == 0)
        return flight_states::EMERGENCY;
    else
        return flight_states::ERROR;
}