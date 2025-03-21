// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bt_plugins/behavior_tree_engine.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

namespace statemachine
{
    BehaviorTreeEngine::BehaviorTreeEngine(const std::vector<std::string> &plugin_libraries)
    {
        BT::SharedLibrary loader;
        for (const auto &p : plugin_libraries)
        {
            factory_.registerFromPlugin(loader.getOSName(p));
        }
    }

    BtStatus
    BehaviorTreeEngine::run(
        BT::Tree *tree,
        std::function<void()> onLoop,
        std::function<bool()> cancelRequested,
        std::chrono::milliseconds loopTimeout)
    {
        rclcpp::WallRate loopRate(loopTimeout);
        BT::NodeStatus result = BT::NodeStatus::RUNNING;

        // Loop until something happens with ROS or the node completes
        try
        {
            while (rclcpp::ok() && result == BT::NodeStatus::RUNNING)
            {
                if (cancelRequested())
                {
                    tree->haltTree();
                    return BtStatus::CANCELED;
                }
                result = tree->tickOnce();

                onLoop();

                loopRate.sleep();
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("BehaviorTreeEngine"),
                "Behavior tree threw exception: %s. Exiting with failure.", ex.what());
            return BtStatus::FAILED;
        }

        return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
    }

    BT::Tree
    BehaviorTreeEngine::createTreeFromText(
        const std::string &xml_string,
        BT::Blackboard::Ptr blackboard,
        std::string tree_name = "MainTree")
    {
        factory_.registerBehaviorTreeFromText(xml_string);
        return factory_.createTree(tree_name, blackboard);
    }

    BT::Tree
    BehaviorTreeEngine::createTreeFromFile(
        const std::string &file_path,
        BT::Blackboard::Ptr blackboard,
        std::string tree_name = "MainTree")
    {
        factory_.registerBehaviorTreeFromFile(file_path);
        return factory_.createTree(tree_name, blackboard);
    }

    // In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
    void
    BehaviorTreeEngine::haltAllActions(BT::TreeNode *root_node)
    {
        if (!root_node)
        {
            return;
        }

        // this halt signal should propagate through the entire tree.
        root_node->haltNode();

        // but, just in case...
        auto visitor = [](BT::TreeNode *node)
        {
            if (node->status() == BT::NodeStatus::RUNNING)
            {
                node->haltNode();
            }
        };
        BT::applyRecursiveVisitor(root_node, visitor);
    }
}