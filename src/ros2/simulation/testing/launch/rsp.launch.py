from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("testron", package_name="testing").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
