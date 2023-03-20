cmake_minimum_required(VERSION 3.5)

project(rmf_robast_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  config/
  model/
  DESTINATION share/${PROJECT_NAME}
)
 set(crowd_sim_config_resource ${output_dir}/config_resource/)

  add_custom_command(
    OUTPUT ${world_name}_crowdsim
    COMMAND ros2 run rmf_building_map_tools building_crowdsim ${map_path} ${crowd_sim_config_resource} ${output_world_path}
    DEPENDS ${output_world_path}
  )


  # generate Ignition simulation world

  set(output_world_name "6og")
  set(ign_output_dir ${CMAKE_CURRENT_BINARY_DIR}/6og_ign)
  set(ign_output_world_path ${ign_output_dir}/${output_world_name}.world)
  set(ign_output_model_dir ${ign_output_dir}/model)
  set(output_model_dir ${output_dir})
  
  add_custom_target(build-time-make-directory ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory ${ign_output_dir})
  
  add_custom_command(
    OUTPUT ${ign_output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator ignition "/workspace/src/rmf/rmf_robast_demo/model/6og.building.yaml" ${map_path} ${ign_output_world_path} ${ign_output_model_dir} 
    DEPENDS ${map_path}
  )

  add_custom_target(generate_map ALL DEPENDS ${map_path} ${ign_output_world_path})
  
  install(
    DIRECTORY ${ign_output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )


ament_package()
