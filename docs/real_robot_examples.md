# Running tentabot on turtlebot 2:

#### 5.1 Turtlebot2: Bring-up the robot:
```
roslaunch tentabot turtlebot2_bringup.launch
```

#### 5.2 Turtlebot2: Start Rviz for visualization:
```
roslaunch tentabot turtlebot2_rviz.launch
```

#### 5.3 Turtlebot2: Start camera:
```
roslaunch tentabot turtlebot2_camera.launch
```

#### 5.4 Turtlebot2: Start laser-scan:
```
roslaunch tentabot sick_lms_1xx.launch
```

#### 5.5 Turtlebot2: Start gmapping:
```
roslaunch tentabot gmapping_sick.launch
```

#### 5.6 Turtlebot2: Start map utility:
```
roslaunch tentabot map_utility_server_turtlebot2.launch
```

##### 5.7 Start Tentabot-DRL service:
```
roslaunch tentabot tentabot_drl_service_turtlebot2.launch
```

##### 5.8 Start training:
```
roslaunch tentabot tentabot_drl_training.launch #Set config file to training_real
```
OR

##### 5.8 Start testing:
```
roslaunch tentabot tentabot_drl_testing.launch #Set config file to testing_real
```

#### (Optional) Turtlebot2: Start teleop:
```
roslaunch turtlebot_teleop keyboard_teleop.launch
