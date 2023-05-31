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
#include <mutex>
#include <queue>
#include <string>
#include <math.h>
#include <iterator>
#include <chrono>

namespace utils
{
    std::vector<std::string> split_space_delimiter(std::string str);
    
    std::vector<std::string> split_colon_delimiter(std::string str);

    std::vector<std::string> split_fullstop_delimiter(std::string str);
}

#endif