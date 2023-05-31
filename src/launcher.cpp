#include <thread>
#include <cstdlib>
#include <vector>
#include <string>
#include <regex>
#include <unistd.h>

#include <fmt/core.h>
#include <fmt/color.h>

#include <signal.h>

#define llog "[launcher]"

void help(const std::string& bin_name)
{
    fmt::print(fg(fmt::color::orange_red),
        "{} {} --help # only valid URL should be given\n", llog, bin_name);
    fmt::print(fg(fmt::color::orange),
        "{} format : {} <connection-url> <sim/real> <number-of-drones>\n", llog, bin_name);
    fmt::print(
        "{} <connection-url> format should be :\n", llog);
    fmt::print(
        "{}     for tcp : tcp://[server_host][:server_port]\n", llog);
    fmt::print(
        "{}     for udp : udp://[bind_host][:bind_port]\n", llog);
    fmt::print(
        "{}     for serial : serial:///path/to/serial/dev[:baudrate]\n", llog);
}

std::vector<std::string> 
    split_colon_delimiter(std::string str)
{
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> output;
    while (std::getline(ss, item, ':'))
        output.push_back(item);
    
    return output;
}

void signal_callback_handler(int signum) 
{
    fmt::print(fg(fmt::color::light_green),
        "{} successfully terminated all programs\n", llog);
    // Terminate program
    exit(signum);
}

std::vector<std::thread> threads;

int main(int argc, char* argv[])
{
    using namespace std::literals::chrono_literals;
    bool is_sim = false;

    // do checks for the executable and arguments
    if (!std::strcmp(argv[2], "sim"))
    {
        if (argc != 4) 
        {
            help(argv[0]);
            return 1;
        }
        is_sim = true;
    }
    else if (!std::strcmp(argv[2], "real"))
    {
        if (argc != 3) 
        {
            help(argv[0]);
            return 1;
        }
    }
    else
    {
        help(argv[0]);
        return 1;
    }

    uint8_t number_of_agents = is_sim ? std::stoi(argv[3]) : 1;
    std::vector<std::string> delimit =
        split_colon_delimiter(argv[1]);
    std::string delimit_wo_port_baud = delimit[0] + ":" + delimit[1] + ":";
    int port_baud = std::stoi(delimit.back().c_str());
    
    for (int i = 0; i < number_of_agents; i++)
    {
        // if fork == 0 meaning child process
        if (fork() == 0) 
        {
            std::string cmd = 
                "./flight_manager " + 
                delimit_wo_port_baud + 
                std::to_string(port_baud + i);
            std::string bin_name = "./flight_manager";
            
            // std::string cmd = 
            //     "./dds_mav_test " + 
            //     delimit_wo_port_baud + 
            //     std::to_string(port_baud + i);
            // std::string bin_name = "./dds_mav_test";
            
            fmt::print(
                "{} launching {}\n", llog, cmd);
            
            std::string url = delimit_wo_port_baud + 
                std::to_string(port_baud + i);
            char *arg[] = {
                const_cast<char*>(bin_name.c_str()), 
                const_cast<char*>(url.c_str()),
                NULL
            };

            // https://www.digitalocean.com/community/tutorials/execvp-function-c-plus-plus
            int results = execvp(bin_name.c_str(), arg);
            fmt::print(fg(fmt::color::light_green),
                "{} exit {}\n", llog, results);
        }
    }
    
    // Register signal and signal handler
    signal(SIGINT, signal_callback_handler);
    fmt::print(fg(fmt::color::light_green),
        "{} registered signit handler\n", llog);

    while (1)
    {
        std::this_thread::sleep_for(100ms);
    }
    
    return 0;
}