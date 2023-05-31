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