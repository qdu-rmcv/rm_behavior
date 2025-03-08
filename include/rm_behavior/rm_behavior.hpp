// Copyright 2025 Jquark
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

#ifndef RM_BEHAVIOR__RM_BEHAVIOR_HPP_
#define RM_BEHAVIOR__RM_BEHAVIOR_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace namespace_name {

class rm_behavior {
public:
    rm_behavior();
    ~rm_behavior();

    static void providedPorts();

    bool setGoal();
    void onHalt();

};

}  // namespace namespace_name

#endif // RM_BEHAVIOR__RM_BEHAVIOR_HPP_