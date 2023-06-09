#include <atomic>
#include <cmath>
#include <iostream>
#include <queue>
#include <thread>
#include <random>
#include <unordered_set>
#include <mutex>

#include <fmt/core.h>
#include <fmt/color.h>

#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

#include "foxglove/PoseInFrame.pb.h"
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include "custom_msgs/msg/AgentStatus.h"
#include "custom_msgs/msg/AgentStatusSubscriber.h"
#include "geometry_msgs/msg/PoseStamped.h"
#include "geometry_msgs/msg/PoseStampedSubscriber.h"

#include "utils.h"

using namespace utils;

namespace foxglove {
template <>
void Server<WebSocketNoTls>::setupTlsHandler() {}
}  // namespace foxglove

std::mutex _mutex;
std::queue<uint8_t> _new_agents;
std::map<uint8_t, eprosima::fastrtps::types::LBoundSeq> _agent_chan;
std::map<uint8_t, toc> _agents_map;
std::unique_ptr<foxglove::Server<foxglove::WebSocketNoTls>> server;

static uint64_t nanosecondsSinceEpoch() {
	return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
						std::chrono::system_clock::now().time_since_epoch())
						.count());
}

// Adapted from:
// https://gist.github.com/tomykaira/f0fd86b6c73063283afe550bc5d77594
// https://github.com/protocolbuffers/protobuf/blob/01fe22219a0312b178a265e75fe35422ea6afbb1/src/google/protobuf/compiler/csharp/csharp_helpers.cc#L346
static std::string Base64Encode(std::string input) {
  constexpr const char ALPHABET[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  // Every 3 bytes of data yields 4 bytes of output
  result.reserve((input.size() + (3 - 1 /* round up */)) / 3 * 4);

  // Unsigned values are required for bit-shifts below to work properly
  const unsigned char* data = reinterpret_cast<const unsigned char*>(input.data());

  size_t i = 0;
  for (; i + 2 < input.size(); i += 3) {
    result.push_back(ALPHABET[data[i] >> 2]);
    result.push_back(ALPHABET[((data[i] & 0b11) << 4) | (data[i + 1] >> 4)]);
    result.push_back(ALPHABET[((data[i + 1] & 0b1111) << 2) | (data[i + 2] >> 6)]);
    result.push_back(ALPHABET[data[i + 2] & 0b111111]);
  }
  switch (input.size() - i) {
    case 2:
      result.push_back(ALPHABET[data[i] >> 2]);
      result.push_back(ALPHABET[((data[i] & 0b11) << 4) | (data[i + 1] >> 4)]);
      result.push_back(ALPHABET[(data[i + 1] & 0b1111) << 2]);
      result.push_back('=');
      break;
    case 1:
      result.push_back(ALPHABET[data[i] >> 2]);
      result.push_back(ALPHABET[(data[i] & 0b11) << 4]);
      result.push_back('=');
      result.push_back('=');
      break;
  }

  return result;
}

// Writes the FileDescriptor of this descriptor and all transitive dependencies
// to a string, for use as a channel schema.
static std::string SerializeFdSet(const google::protobuf::Descriptor* toplevelDescriptor) {
	google::protobuf::FileDescriptorSet fdSet;
	std::queue<const google::protobuf::FileDescriptor*> toAdd;
	toAdd.push(toplevelDescriptor->file());
	std::unordered_set<std::string> seenDependencies;
	while (!toAdd.empty()) {
		const google::protobuf::FileDescriptor* next = toAdd.front();
		toAdd.pop();
		next->CopyTo(fdSet.add_file());
		for (int i = 0; i < next->dependency_count(); ++i) {
		const auto& dep = next->dependency(i);
		if (seenDependencies.find(dep->name()) == seenDependencies.end()) {
			seenDependencies.insert(dep->name());
			toAdd.push(dep);
		}
		}
  }
  return fdSet.SerializeAsString();
}

void agent_status_handler()
{
    AgentStatusSubscriber status_subscriber;
    if (!status_subscriber.init("rt/agents/status"))
        return;
        
    double user_frequency = 20.0;
    while (1)
    {
        custom_msgs::msg::AgentStatus _agent_status = 
            status_subscriber.run(user_frequency);
        
		_mutex.lock();
        // when we receive a command we should handle it here
        if (_agents_map.empty())
        {
			toc _toc;
			_toc.sys_id = _agent_status.mav_id().data();
			_toc.flight_mode = get_fc_flight_mode(_agent_status.fm_state().data());
			_toc.flight_state = get_flight_state(_agent_status.fc_state().data());
			_agents_map.insert(
				std::pair<uint8_t, toc>(
					_agent_status.mav_id().data(), _toc));
			_new_agents.push(_agent_status.mav_id().data());
			_mutex.unlock();
			continue;
        }

        auto iterator = _agents_map.find(_agent_status.mav_id().data());
        if (iterator == _agents_map.end())
        {
			toc _toc;
			_toc.sys_id = _agent_status.mav_id().data();
			_toc.flight_mode = get_fc_flight_mode(_agent_status.fm_state().data());
			_toc.flight_state = get_flight_state(_agent_status.fc_state().data());
			_agents_map.insert(
				std::pair<uint8_t, toc>(
					_agent_status.mav_id().data(), _toc));
			_new_agents.push(_agent_status.mav_id().data());
			_mutex.unlock();
			continue;
        }
		_mutex.unlock();

        iterator->second.sys_id = 
			_agent_status.mav_id().data();
		iterator->second.flight_mode = 
			get_fc_flight_mode(_agent_status.fm_state().data());
		iterator->second.flight_state = 
			get_flight_state(_agent_status.fc_state().data());
		
    }
}

void agent_pose_handler(double freq)
{
    std::map<uint8_t, std::shared_ptr<PoseStampedSubscriber>> pose_subscribers;
    
    int timer_milliseconds = (int)std::floor(1 / freq * 1000.0);
    
    while (1)
    {
        // check agent map to see if there is increment of size
        _mutex.lock();
        while (!_new_agents.empty())
        {
			std::shared_ptr<PoseStampedSubscriber> _pss =
				std::make_shared<PoseStampedSubscriber>(PoseStampedSubscriber());
			if (!_pss->init("rt/mav" + std::to_string(_new_agents.front()) + "/pose"))
			{
				_new_agents.pop();
				continue;
			}
			// update subscriber list
			pose_subscribers.insert(
				std::pair<uint8_t, std::shared_ptr<PoseStampedSubscriber>>(
				_new_agents.front(), _pss
				)
			);
			const auto chanelIds = server->addChannels({{
				.topic = "mav" + std::to_string(_new_agents.front()) + "/pose",
				.encoding = "protobuf",
				.schemaName = foxglove::PoseInFrame::descriptor()->full_name(),
				.schema = Base64Encode(SerializeFdSet(foxglove::PoseInFrame::descriptor())),
			}});
			
			_agent_chan.insert(
				std::pair<int8_t, eprosima::fastrtps::types::LBoundSeq>(
					_new_agents.front(), chanelIds));
			_new_agents.pop();
        }
        _mutex.unlock();

        if (pose_subscribers.empty())
          continue;

        for (auto &sub : pose_subscribers)
        {
			geometry_msgs::msg::PoseStamped data;

			if (sub.second->run(data))
			{
				auto iterator = _agents_map.find(sub.first);
				if (iterator == _agents_map.end()) continue;
				auto iterator_chan = _agent_chan.find(sub.first);
				if (iterator_chan == _agent_chan.end()) continue;

				const auto now = nanosecondsSinceEpoch();
				foxglove::PoseInFrame msg;
				msg.set_frame_id("world");
				auto* position = msg.mutable_pose()->mutable_position();
				auto* orientation = msg.mutable_pose()->mutable_orientation();

				position->set_x(data.pose().position().x());
				position->set_y(data.pose().position().y());
				position->set_z(data.pose().position().z());

				const auto serializedMsg = msg.SerializeAsString();
				server->broadcastMessage(iterator_chan->second.front(), now, reinterpret_cast<const uint8_t*>(serializedMsg.data()),
										serializedMsg.size());
			}
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(timer_milliseconds));
    }
}

int main() 
{
    std::vector<std::thread> threads;

    const auto logHandler = [](foxglove::WebSocketLogLevel, char const* msg) {
        std::cout << msg << std::endl;
    };
    foxglove::ServerOptions serverOptions;
    // foxglove::Server<foxglove::WebSocketNoTls> server;
    server = std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(
        "C++ [Protobuf] server", logHandler, serverOptions);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;
    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle) {
        std::cout << "first client subscribed to " << chanId << std::endl;
    };
    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle) {
        std::cout << "last client unsubscribed from " << chanId << std::endl;
    };
    server->setHandlers(std::move(hdlrs));
    server->start("0.0.0.0", 8765);

    bool running = true;

    server->getEndpoint();

    boost::asio::signal_set signals(server->getEndpoint().get_io_service(), SIGINT);
    signals.async_wait([&](std::error_code const& ec, int sig) 
    {
        if (ec) {
            std::cerr << "signal error: " << ec.message() << std::endl;
            return;
        }
        std::cerr << "received signal " << sig << ", shutting down" << std::endl;
        running = false;
    });

    // create a thread for agent status dds subscriber
    std::thread t_d_as(&agent_status_handler);
    threads.push_back(std::move(t_d_as));

    // create a thread for agent pose dds subscriber
    std::thread t_d_p(&agent_pose_handler, 2.0);
    threads.push_back(std::move(t_d_p));

    auto start = std::chrono::system_clock::now();

    while(running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

	for (auto const& [key, val] : _agent_chan)
		server->removeChannels(val);
    
    server->stop();

    return 0;
}