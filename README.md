# ARL_kalman_hand_tracking
 
**build:**

`colcon build`

`source install/setup.bash`

**Terminal 1:** Running camera node
```bash
ros2 run kalman_hand_tracking camera_node
```
**Terminal 2:** Runnig main program
```bash
ros2 run kalman_hand_tracking drone_control_node
```
