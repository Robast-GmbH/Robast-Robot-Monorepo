#ifndef BT_BASICS__BT_PLUGIN_MUXOR_HPP_
#define BT_BASICS__BT_PLUGIN_MUXOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bt_basics
{
    class PluginMuxer {
    public:
        PluginMuxer():current_actor_(std::string("")) {}

        void startActor(const std::string& actor_name)
        {
            std::scoped_lock l(mutex_);
            if (current_actor_ != actor_name)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("NavigatorMutex"),
                    "Major error! Navigation stopped while another navigation"
                    " task is in progress! This likely occurred from an incorrect"
                    "implementation of a navigator plugin.");
            }
            else
            {
                current_actor_ = std::string("");
            }

        }
    protected:
        std::string current_actor_;
        std::mutex mutex_;
    };

    template<class ActionT>
    class BaseActor {
    public:
        using Ptr = std::shared_ptr<bt_basics::BaseActor<ActionT>>;

        BaseActor()
        {
            plugin_muxer_ = nullptr;
        }
    protected:
        NavigatorMuxer * plugin_muxer_;
    };
}

#endif