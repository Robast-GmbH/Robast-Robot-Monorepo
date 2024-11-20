/*
Project error handles:
    Communication Interfaces: 01
    error_handling: 02
    drawer_bridge: 03
    dryve_d1_bridge: 04
    nfc_bridge: 05
    web_bridge_nfc: 06
    nav_bringup: 10
    nav_bt_plugins: 11
    nav_recoveries: 12
    map_update_module: 13
    split_path_to_goal: 14
    robot_description: 15
    simple_fleetmanagement: 20
    theron_fleetmanagement_bridge: 21
    statemachine: 30
    bt_base_nodes: 31
    bt_conversions: 32
    bt_plugins: 33
    drawer_sm: 34
    simulation: 80
    drawer_bridge_simulation: 81
    gazebo_controller_manager: 82
    moveit_simulation: 83
    tiplu_world: 84

*/

// debug_codes
// first digit is a 1
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error

// trace_codes
// first digit is a 2
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error

// info_codes
// first digit is a 3
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error

// warning_codes
// first digit is a 4
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error

// error_codes
// first digit is a 5
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error
#define ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED                              50301
#define ERROR_CODES_TIMEOUT_DRAWER_NOT_OPENED_INTERFACE                    communication_interfaces::msg::DrawerAddress
#define ERROR_CODES_DRAWER_CLOSED_IN_IDLE_STATE                            50302
#define ERROR_CODES_DRAWER_CLOSED_IN_IDLE_STATE_INTERFACE                  communication_interfaces::msg::DrawerAddress
#define ERROR_CODES_MOTOR_DRIVER_CONTROL_NOT_SUPPORTED_BY_MODULE           50303
#define ERROR_CODES_MOTOR_DRIVER_CONTROL_NOT_SUPPORTED_BY_MODULE_INTERFACE communication_interfaces::msg::DrawerAddress

// fatal_codes
// first digit is a 6
// 2nd and 3rd digits indicates the project
// 4th and 5th digits indicates the specific error
