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

        bool isActive()
        {
            std::scoped_lock l(mutex_);
            return !current_actor_.empty();
        }

        
        void startActor(const std::string& actor_name)
        {
            std::scoped_lock l(mutex_);
            if (current_actor_ != actor_name)
            {
                RCLCPP_ERROR(
                    rclcpp::get_logger("PluginMuxer"),
                    "Major error! Action stopped due to a different action intersecting the process"
                    "This likely occurred from an incorrect"
                    "implementation of an actor plugin.");
            }
            else
            {
                current_actor_ = std::string("");
            }
        }

        void stopActor(const std::string & actor_name)
        {
            std::scoped_lock l(mutex_);
            if (current_actor_ != actor_name) {
                RCLCPP_ERROR(
                        rclcpp::get_logger("PluginMuxer"),
                        "Major error! Action stopped due to a different action intersecting the process"
                        "This likely occurred from an incorrect"
                        "implementation of an actor plugin.");
            } else {
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
    
        bool on_configure(
            rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
            const std::vector<std::string> & plugin_lib_names,
            bt_basics::PluginMuxer * plugin_muxer)
        {
            auto node = parent_node.lock();
            logger_ = node->get_logger();
            clock_ = node->get_clock();
            plugin_muxer_ = plugin_muxer;

            // get the default behavior tree for this 
            std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

            // Create the Behavior Tree Action Server for this 
            bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
                node,
                getName(),
                plugin_lib_names,
                default_bt_xml_filename,
                std::bind(&BaseActor::onGoalReceived, this, std::placeholders::_1),
                std::bind(&BaseActor::onLoop, this),
                std::bind(&BaseActor::onPreempt, this, std::placeholders::_1),
                std::bind(&BaseActor::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

            bool ok = true;
            if (!bt_action_server_->on_configure()) {
                ok = false;
            }
            BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();

            return configure(parent_node) && ok;
        }
        bool on_activate()
        {
            bool ok = true;

            if (!bt_action_server_->on_activate()) {
            ok = false;
            }

            return activate() && ok;
        }

        bool on_deactivate()
        {
            bool ok = true;
            if (!bt_action_server_->on_deactivate()) {
            ok = false;
            }

            return deactivate() && ok;
        }

        /**
         * @brief Cleanup a navigator
         * @return bool If successful
         */
        bool on_cleanup()
        {
            bool ok = true;
            if (!bt_action_server_->on_cleanup()) {
            ok = false;
            }

            bt_action_server_.reset();

            return cleanup() && ok;
        }

        /**
         * @brief Get the action name of this navigator to expose
         * @return string Name of action to expose
         */
        virtual std::string getName() = 0;

        virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

        /**
         * @brief Get the action server
         * @return Action server pointer
         */
        //TODO nix nav
        std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
        {
            return bt_action_server_;
        }

        virtual ~BaseActor() = default;
        
    protected:
        bool onGoalReceived(typename ActionT::ConstSharedPtr goal)
        {//TODO nix nav
            if (plugin_muxer_->isActive()) {
            RCLCPP_ERROR(
                logger_,
                "Requested navigation from %s while another actor is processing,"
                " rejecting request.", getName().c_str());
            return false;
            }

                plugin_muxer_->startActor(getName());
            

            return goal_accepted;
        }

        /**
         * @brief An intermediate completion function to mux actors
         */
        //TODO nix nav
        void onCompletion(
            typename ActionT::Result::SharedPtr result,
            const nav2_behavior_tree::BtStatus final_bt_status)
        {
            plugin_muxer_->stopActor(getName());
            actionCompleted(result, final_bt_status);
        }

        /**
         * @brief A callback that defines execution that happens on one iteration through the BT
         * Can be used to publish action feedback
         */
        virtual void onLoop() = 0;

        /**
         * @brief A callback that is called when a preempt is requested
         */
        virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

        /**
         * @brief A callback that is called when a the action is completed; Can fill in
         * action result message or indicate that this action is done.
         */
        virtual void actionCompleted(
            typename ActionT::Result::SharedPtr result,
            const nav2_behavior_tree::BtStatus final_bt_status) = 0;

        /**
         * @param Method to configure resources.
         */
        virtual bool configure(
            rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/)
        {
            return true;
        }

        /**
         * @brief Method to cleanup resources.
         */
        virtual bool cleanup() {return true;}

        /**
         * @brief Method to activate any threads involved in execution.
         */
        virtual bool activate() {return true;}

        /**
         * @brief Method to deactivate and any threads involved in execution.
         */
        virtual bool deactivate() {return true;}

        
        PluginMuxer* plugin_muxer_;
        rclcpp::Logger logger_{ rclcpp::get_logger("Actor") };
        rclcpp::Clock::SharedPtr clock_;
    };
    
}

#endif