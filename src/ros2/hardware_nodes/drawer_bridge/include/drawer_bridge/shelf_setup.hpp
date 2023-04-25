#include <iostream>

#include "communication_interfaces/msg/module.hpp"
#include "drawer_defines.h"

class ShelfSetup {
public:
    static std::vector<communication_interfaces::msg::Module> get_all_mounted_drawers()
    {
        // TODO: This should actually be done automatically by polling all drawer_controller on the CAN bus
        communication_interfaces::msg::Box box_10x40x1;
        box_10x40x1.x = DRAWER_INSIDE_WIDTH_10x40x1;
        box_10x40x1.y = DRAWER_INSIDE_DEPTH_10x40x1;
        box_10x40x1.z = DRAWER_INSIDE_HEIGHT_10x40x1;

        communication_interfaces::msg::Box box_20x40x1;
        box_20x40x1.x = DRAWER_INSIDE_WIDTH_20x40x1;
        box_20x40x1.y = DRAWER_INSIDE_DEPTH_20x40x1;
        box_20x40x1.z = DRAWER_INSIDE_HEIGHT_20x40x1;

        communication_interfaces::msg::Box box_30x40x1;
        box_30x40x1.x = DRAWER_INSIDE_WIDTH_30x40x1;
        box_30x40x1.y = DRAWER_INSIDE_DEPTH_30x40x1;
        box_30x40x1.z = DRAWER_INSIDE_HEIGHT_30x40x1;

        communication_interfaces::msg::Module drawer_1;
        drawer_1.module_id = 1;
        drawer_1.number_of_drawers = 1;
        drawer_1.drawer_size = box_10x40x1;

        communication_interfaces::msg::Module drawer_2;
        drawer_2.module_id = 2;
        drawer_2.number_of_drawers = 1;
        drawer_2.drawer_size = box_10x40x1;

        communication_interfaces::msg::Module drawer_3;
        drawer_3.module_id = 3;
        drawer_3.number_of_drawers = 1;
        drawer_3.drawer_size = box_10x40x1;

        communication_interfaces::msg::Module drawer_4;
        drawer_4.module_id = 4;
        drawer_4.number_of_drawers = 1;
        drawer_4.drawer_size = box_20x40x1;

        communication_interfaces::msg::Module drawer_5;
        drawer_5.module_id = 5;
        drawer_5.number_of_drawers = 1;
        drawer_5.drawer_size = box_30x40x1;

        return { drawer_1, drawer_2, drawer_3, drawer_4, drawer_5 };
    };
};
