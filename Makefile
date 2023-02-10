all: build

build:
	docker exec -it ros_turtlebot_sim bash -c "source /ros_entrypoint.sh && cd /root/ros_ws && colcon build"

run:
	docker exec -it ros_tools bash -c "source /ros_entrypoint.sh && source /root/ros_ws/install/setup.bash && ros2 run turtle_control_Higor turtle_control"

# 	docker exec -it ros_tools bash -c "source /ros_entrypoint.sh && source /root/ros_ws/install/setup.bash && ros2 run turtle_control_${TURTLE_CONTROL_NAME} turtle_control"
# publish:
#     docker exec -it ros_tools bash -c "source /ros_entrypoint.sh && ros2 topic pub -1 /HDO/goal geometry_msgs/msg/Pose2D \"{x: 2.0, y: 2.0, theta: 0.0}\" "