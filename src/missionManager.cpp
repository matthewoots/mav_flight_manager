#include <thread>
#include <cstdlib>
#include <iostream>
#include <regex>
#include <unistd.h>
#include <experimental/filesystem>

#include <yaml-cpp/yaml.h>

#include "utils.h"

#include <fmt/core.h>
#include <fmt/color.h>

#define mlog "[mission]"
using namespace utils;
namespace fes = std::experimental::filesystem;

bool try_stoi(int &i, const std::string &s)
{
    try 
    {
        i = std::stoi(s);
        return true;
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
}

std::map<std::string, custom_commands> _cmd_dict;

int main(int argc, char* argv[])
{
    if (argc > 2)
        exit(EXIT_FAILURE);

    auto path = fes::current_path();
    auto config_path = path.string() + "/config/mission";
    uint8_t count = 0;
    std::map<uint8_t, std::string> _path_map;
    for (const auto & entry : fes::directory_iterator(config_path))
    {
        fmt::print("{} {}. {} \n", mlog, 
            count, entry.path().filename().string());
        
        _path_map.insert(
            std::pair<uint8_t, std::string>(
                count, entry.path().string()));
        count++;
    }

    // wait for response
    std::string selection;
    std::cin >> selection;

    auto sel = _path_map.find(std::stoi(selection));

    if (sel == _path_map.end())
        exit(EXIT_FAILURE);

    if (!fes::exists(sel->second))
        exit(EXIT_FAILURE);

    fmt::print(fg(fmt::color::light_green), 
        "{} opening {} ...\n", mlog, sel->second);
    YAML::Node mission = YAML::LoadFile(sel->second);

    fmt::print("{} mission with {} custom commands\n", 
        mlog, mission["custom_commands"].size());
    
    // evaluate and save the custom commands
    for (YAML::iterator it = mission["custom_commands"].begin();
        it != mission["custom_commands"].end();
        it++) 
    {
        std::vector<std::string> str_vect = 
            it->second.as<std::vector<std::string>>();
        // reject commands if they are not in format
        if (str_vect.size() > 4 && str_vect.size() < 3)
            continue;
        
        std::pair<std::string, custom_commands> _tmp;
        _tmp.first = it->first.as<std::string>();
        
        custom_commands cc;
        int8_t result_command = 
            cc.string_to_cmd_uint(str_vect[0]);
        int8_t result_sync = 
            cc.string_to_sync_uint(str_vect[1]);
        
        if (result_command == -1 || result_sync == -1)
            continue;

        // if participants are not equal to all 
        if (std::strcmp("ALL", str_vect[2].c_str()) != 0)
        {
            std::vector<std::string> participants_vector =
                split_space_delimiter(str_vect[2]);
            for (auto parti : participants_vector)
            {
                int i;
                if (try_stoi(i, parti))
                    cc.participants.emplace_back(i);
            }
        }
        else
            cc.participants.emplace_back(-1);

        cc.command = result_command;
        cc.sync_mode = result_sync;
        cc.duration = 0.0;

        if (result_command == cc.CHOLD)
            cc.duration = std::stod(str_vect[3]);
        else if (result_command == cc.CMOVE)
        {
            std::vector<std::string> target_vector =
                split_space_delimiter(str_vect[3]);
            if (target_vector.size() % 4 != 0)
                continue;
            int target_size = target_vector.size() / 4;

            for (int n = 0; n < target_size; n++)
            {
                Eigen::Vector4f vf;
                for (int c = 0; c < 4; c++)
                    vf[c] = std::stof(target_vector[n*4+c]);
                cc.target.emplace_back(vf);
            }
        }

        fmt::print("{} name:{}, cmd:{}, sync:{}, part:", 
            mlog, _tmp.first, cc.command, cc.sync_mode);
        if (cc.participants.empty())
            fmt::print("[]");
        else
            for (int n = 0; n < cc.participants.size(); n++)
            {
                fmt::print("{}", cc.participants[n]);
                if (n != cc.participants.size()-1)
                    fmt::print(" ");
            }

        fmt::print(", dur:{}, target:", cc.duration);

        for (Eigen::Vector4f targ : cc.target)
            fmt::print("({} {} {} {}) ", 
               targ[0], targ[1], targ[2], targ[3]);           

        fmt::print("\n");
    }

    // spend 3secs scanning for mavs
    

    // go through the command sequence and add in the commands
    
    
    exit(EXIT_SUCCESS);
}