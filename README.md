### VicPinky의 ROS 프로젝트를 CPP로 변환한 레포지토리입니다.

```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
// domain id 설정 (상황에 따라 설정할 것)
// venv 설정 (상황에 따라 설정할 것)
```

실행

[robot]
- **bringup** : ros2 launch vicpinky_bringup bringup.launch.xml
- **slam 실행** : ros2 launch vicpinky_navigation map_building.launch.xml
- **맵 저장** : ros2 run nav2_map_server map_saver_cli -f "저장할 맵 이름"
- **네비게이션 실행** : ros2 launch vicpinky_navigation bringup_launch.xml map:=저장한_맵.yaml

[pc]
- **gazebo** ros2 launch vicpinky_bringup gazebo_bringup.launch.xml
- **rviz** : ros2 launch vicpinky_navigation map_view.launch.xml
- **키조작** : ros2 run teleop_twist_keyboard teleop_twist_keyboard
- **rviz 실행** : ros2 launch vicpinky_navigation nav2_view.launch.xml