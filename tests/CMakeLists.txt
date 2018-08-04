
catkin_add_gtest(mujoco mujoco.cpp)
target_link_libraries(mujoco ${catkin_LIBRARIES} ${mujoco_LIBRARIES})
add_dependencies(mujoco ${catkin_EXPORTED_TARGETS})
