import os
import sys

current_script_dir = os.path.dirname(__file__)
workspace_dir = os.path.abspath(os.path.join(current_script_dir, "..", "..", "..", "..", ".."))
sys.path.append(os.path.join(workspace_dir, "build", "can"))  # Add the build directory to the Python path
import can_db_defines_bindings as can_db_defines

from can_msgs.msg import Frame


def construct_e_drawer_motor_control_feedback_can_frame(electrical_drawer_motor_control_goal):
    e_motor_control_changed = 1
    expected_data_motor_control_feedback_uint64 = (
        (
            electrical_drawer_motor_control_goal.module_address.module_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID
            )
        )
        | (
            electrical_drawer_motor_control_goal.motor_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID
            )
        )
        | (
            electrical_drawer_motor_control_goal.enable_motor
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR
            )
        )
        | (
            e_motor_control_changed
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_CONFIRM_CHANGE
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_CONFIRM_CHANGE
            )
        )
    )
    electrical_drawer_motor_control_feedback = Frame()
    electrical_drawer_motor_control_feedback.id = can_db_defines.can_id.ELECTRICAL_DRAWER_MOTOR_CONTROL
    electrical_drawer_motor_control_feedback.dlc = can_db_defines.can_dlc.ELECTRICAL_DRAWER_MOTOR_CONTROL
    electrical_drawer_motor_control_feedback.data = list(
        expected_data_motor_control_feedback_uint64.to_bytes(8, byteorder="big")
    )
    return electrical_drawer_motor_control_feedback


def construct_heartbeat_can_frame(module_id, interval_in_ms):
    heartbeat_can_msg = Frame()
    heartbeat_can_msg.id = can_db_defines.can_id.HEARTBEAT
    heartbeat_can_msg.dlc = can_db_defines.can_dlc.HEARTBEAT
    heartbeat_can_msg.data = list(construct_can_data_heartbeat(module_id, interval_in_ms))
    return heartbeat_can_msg


def construct_can_data_drawer_unlock(module_id, drawer_id):
    data_uint64 = (
        module_id
        << (
            can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
            - can_db_defines.can_signal.bit_length.DRAWER_UNLOCK_MODULE_ID
            - can_db_defines.can_signal.bit_start.DRAWER_UNLOCK_MODULE_ID
        )
    ) | (
        drawer_id
        << (
            can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
            - can_db_defines.can_signal.bit_length.DRAWER_UNLOCK_DRAWER_ID
            - can_db_defines.can_signal.bit_start.DRAWER_UNLOCK_DRAWER_ID
        )
    )

    return data_uint64.to_bytes(8, byteorder="big")


def construct_can_data_module_config(module_id, config_id, config_value):
    data_uint64 = (
        (
            module_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.MODULE_CONFIG_MODULE_ID
                - can_db_defines.can_signal.bit_start.MODULE_CONFIG_MODULE_ID
            )
        )
        | (
            config_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.MODULE_CONFIG_CONFIG_ID
                - can_db_defines.can_signal.bit_start.MODULE_CONFIG_CONFIG_ID
            )
        )
        | (
            config_value
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.MODULE_CONFIG_CONFIG_VALUE
                - can_db_defines.can_signal.bit_start.MODULE_CONFIG_CONFIG_VALUE
            )
        )
    )
    return data_uint64.to_bytes(8, byteorder="big")


def construct_can_data_e_drawer_motor_control(module_id, motor_id, enable_motor):
    data_uint64 = (
        (
            module_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_MODULE_ID
            )
        )
        | (
            motor_id
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_MOTOR_ID
            )
        )
        | (
            enable_motor
            << (
                can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
                - can_db_defines.can_signal.bit_length.ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR
                - can_db_defines.can_signal.bit_start.ELECTRICAL_DRAWER_MOTOR_CONTROL_ENABLE_MOTOR
            )
        )
    )
    return data_uint64.to_bytes(8, byteorder="big")


def construct_can_data_heartbeat(module_id, interval_in_ms):
    data_uint64 = (
        module_id
        << (
            can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
            - can_db_defines.can_signal.bit_length.HEARTBEAT_MODULE_ID
            - can_db_defines.can_signal.bit_start.HEARTBEAT_MODULE_ID
        )
    ) | (
        interval_in_ms
        << (
            can_db_defines.CAN_STD_MSG_DLC_MAXIMUM * 8
            - can_db_defines.can_signal.bit_length.HEARTBEAT_INTERVAL_IN_MS
            - can_db_defines.can_signal.bit_start.HEARTBEAT_INTERVAL_IN_MS
        )
    )

    return data_uint64.to_bytes(8, byteorder="big")
