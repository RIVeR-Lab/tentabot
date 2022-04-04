#!/usr/bin/env python3

'''
LAST UPDATE: 2022.04.03

AUTHOR: Neset Unver Akmandor (NUA)
        Gary M. Lvov (GML)

E-MAIL: akmandor.n@northeastern.edu
        lvov.g@northeastern.edu
'''
 
import rospy
import rospkg
import roslaunch 

if __name__=="__main__":

    rospy.init_node("tentabot_framework_launch", anonymous=False)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    rospack = rospkg.RosPack()
    tentabot_path = rospack.get_path('tentabot') + "/"
    #print("game_launch::__main__ -> tentabot_path: " + str(tentabot_path))

    tentabot_launch_path = tentabot_path + "launch/"

    ## General Parameters
    sim_flag = rospy.get_param('sim_flag', True)
    rviz_flag = rospy.get_param('rviz_flag', True)
    map_utility_flag = rospy.get_param('map_utility_flag', True)
    tentabot_server_flag = rospy.get_param('tentabot_server_flag', True)

    print("tentabot_framework_launch:: __main__ -> General Parameters:")
    print("tentabot_framework_launch:: __main__ -> sim_flag: " + str(sim_flag))
    print("tentabot_framework_launch:: __main__ -> rviz_flag: " + str(rviz_flag))
    print("tentabot_framework_launch:: __main__ -> map_utility_flag: " + str(map_utility_flag))
    print("tentabot_framework_launch:: __main__ -> tentabot_server_flag: " + str(tentabot_server_flag))

    ## Task-Nav Parameters
    world_name = rospy.get_param('world_name', "")
    world_frame_name = rospy.get_param('world_frame_name', "")

    print("tentabot_framework_launch:: __main__ -> Task-Nav Parameters:")
    print("tentabot_framework_launch:: __main__ -> world_name: " + str(world_name))
    print("tentabot_framework_launch:: __main__ -> world_frame_name: " + str(world_frame_name))

    n_goal = rospy.get_param('n_goal', "")
    print("tentabot_framework_launch:: __main__ -> n_goal: " + str(n_goal))
    
    goals_x = []
    goals_y = []
    goals_z = []
    goals_yaw = []

    for i in range(1, n_goal+1):

        goal_name = "goal" + str(i)

        goal_x = rospy.get_param(goal_name + '_x', 0.0)
        goal_y = rospy.get_param(goal_name + '_y', 0.0)
        goal_z = rospy.get_param(goal_name + '_z', 0.0)
        goal_yaw = rospy.get_param(goal_name + '_yaw', 0.0)

        goals_x.append(goal_x)
        goals_y.append(goal_y)
        goals_z.append(goal_z)
        goals_yaw.append(goal_yaw)

        print("tentabot_framework_launch:: __main__ -> " + goal_name + ": " + str(goal_x))
        print("tentabot_framework_launch:: __main__ -> " + goal_name + ": " + str(goal_y))
        print("tentabot_framework_launch:: __main__ -> " + goal_name + ": " + str(goal_z))
        print("tentabot_framework_launch:: __main__ -> " + goal_name + ": " + str(goal_yaw))

    robot_name = rospy.get_param('robot_name', "")
    robot_model = rospy.get_param('robot_model', "")
    robot_init_pos_x = rospy.get_param('robot_init_pos_x', 0.0)
    robot_init_pos_y = rospy.get_param('robot_init_pos_y', 0.0)
    robot_init_pos_z = rospy.get_param('robot_init_pos_z', 0.0)
    robot_init_yaw = rospy.get_param('robot_init_yaw', 0.0)
    robot_odometry_msg = rospy.get_param('robot_odometry_msg', "")
    robot_pose_control_msg = rospy.get_param('robot_pose_control_msg', "")
    robot_velo_control_msg = rospy.get_param('robot_velo_control_msg', "")

    print("tentabot_framework_launch:: __main__ -> robot_name: " + str(robot_name))
    print("tentabot_framework_launch:: __main__ -> robot_model: " + str(robot_model))
    print("tentabot_framework_launch:: __main__ -> robot_init_pos_x: " + str(robot_init_pos_x))
    print("tentabot_framework_launch:: __main__ -> robot_init_pos_y: " + str(robot_init_pos_y))
    print("tentabot_framework_launch:: __main__ -> robot_init_pos_z: " + str(robot_init_pos_z))
    print("tentabot_framework_launch:: __main__ -> robot_init_yaw: " + str(robot_init_yaw))
    print("tentabot_framework_launch:: __main__ -> robot_odometry_msg: " + str(robot_odometry_msg))
    print("tentabot_framework_launch:: __main__ -> robot_pose_control_msg: " + str(robot_pose_control_msg))
    print("tentabot_framework_launch:: __main__ -> robot_velo_control_msg: " + str(robot_velo_control_msg))

    ## Map Utility Parameters
    config_map_utility = rospy.get_param('config_map_utility', "")

    print("tentabot_framework_launch:: __main__ -> Map Utility Parameters:")
    print("tentabot_framework_launch:: __main__ -> config_map_utility: " + str(config_map_utility))

    ## Tentabot Server Parameters
    config_tentabot_server = rospy.get_param('config_tentabot_server', "")

    print("tentabot_framework_launch:: __main__ -> Tentabot-Server Parameters:")
    print("tentabot_framework_launch:: __main__ -> config_tentabot_server: " + str(config_tentabot_server))

    ## Launch Simulation
    sim_paused = "false"

    if sim_flag:
        
        if robot_name == "firefly":

            # Launch Mav-Garden
            mav_garden_path = tentabot_launch_path + "utilities/mav_garden.launch"
            mav_garden_args = [mav_garden_path,
                               'world_name:=' + str(world_name),
                               'world_frame_name:=' + str(world_frame_name),
                               'robot_name:=' + str(robot_name),
                               'robot_model:=' + str(robot_model),
                               'robot_init_pos_x:=' + str(robot_init_pos_x),
                               'robot_init_pos_y:=' + str(robot_init_pos_y),
                               'robot_init_pos_z:=' + str(robot_init_pos_z),
                               'robot_odometry_msg:=' + str(robot_odometry_msg),
                               'robot_pose_control_msg:=' + str(robot_pose_control_msg)]

            mav_garden_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mav_garden_args)[0], mav_garden_args[1:]) ]
            mav_garden = roslaunch.parent.ROSLaunchParent(uuid, mav_garden_launch)
            mav_garden.start()

            print("tentabot_framework_launch:: __main__ -> Launched Mav-Garden!")

        else:

            if robot_name == "stretch":
                sim_paused = "true"

            # Launch Mobile-Garden
            mobile_garden_path = tentabot_launch_path + "utilities/mobile_garden.launch"
            mobile_garden_args = [mobile_garden_path,
                                  'sim_paused:=' + str(sim_paused),
                                  'world_name:=' + str(world_name),
                                  'robot_name:=' + str(robot_name),
                                  'robot_model:=' + str(robot_model),
                                  'robot0_init_pos_x:=' + str(robot_init_pos_x),
                                  'robot0_init_pos_y:=' + str(robot_init_pos_y),
                                  'robot0_init_yaw:=' + str(robot_init_yaw)]

            mobile_garden_launch = [ (roslaunch.rlutil.resolve_launch_arguments(mobile_garden_args)[0], mobile_garden_args[1:]) ]
            mobile_garden = roslaunch.parent.ROSLaunchParent(uuid, mobile_garden_launch)
            mobile_garden.start()

            print("tentabot_framework_launch:: __main__ -> Launched Mobile-Garden!")
        
        rospy.sleep(2)

    ## Launch Rviz
    if rviz_flag:

        rviz_path = tentabot_launch_path + "utilities/rviz.launch"
        rviz_args = [rviz_path,
                     'robot_name:=' + str(robot_name)]

        rviz_launch = [ (roslaunch.rlutil.resolve_launch_arguments(rviz_args)[0], rviz_args[1:]) ]
        rviz = roslaunch.parent.ROSLaunchParent(uuid, rviz_launch)
        rviz.start()

        print("tentabot_framework_launch:: __main__ -> Launched Rviz!")
        rospy.sleep(1)

    ## Launch Map Utility Server
    if map_utility_flag:

        map_utility_path = tentabot_launch_path + "utilities/map_utility_server.launch"
        map_utility_args = [map_utility_path,
                            'config_map_utility:=' + str(config_map_utility)]

        map_utility_launch = [ (roslaunch.rlutil.resolve_launch_arguments(map_utility_args)[0], map_utility_args[1:]) ]
        map_utility = roslaunch.parent.ROSLaunchParent(uuid, map_utility_launch)
        map_utility.start()

        print("tentabot_framework_launch:: __main__ -> Launched Map Utility Server!")
        rospy.sleep(1)
    
    ## Launch Tentabot Server
    if tentabot_server_flag:

        tentabot_server_path = tentabot_launch_path + "tentabot_server.launch"
        tentabot_server_args = [tentabot_server_path,
                                   'config_tentabot_server:=' + str(config_tentabot_server)]

        tentabot_server_launch = [ (roslaunch.rlutil.resolve_launch_arguments(tentabot_server_args)[0], tentabot_server_args[1:]) ]
        tentabot_server = roslaunch.parent.ROSLaunchParent(uuid, tentabot_server_launch)
        tentabot_server.start()

        print("tentabot_framework_launch:: __main__ -> Launched Tentabot Server!")
        rospy.sleep(1)

    ## Launch Tentabot-DRL Training/Testing
    drl_service_flag = rospy.get_param('drl_service_flag', False)
    mode = rospy.get_param('mode', "")
    
    print("tentabot_framework_launch:: __main__ -> drl_service_flag: " + str(drl_service_flag))
    print("tentabot_framework_launch:: __main__ -> mode: " + str(mode))
    
    if drl_service_flag:

        tentabot_drl_path = tentabot_launch_path + "tentabot_drl/tentabot_drl_" + mode + ".launch"
        tentabot_drl_args = [tentabot_drl_path]

        tentabot_drl_launch = [ (roslaunch.rlutil.resolve_launch_arguments(tentabot_drl_args)[0]) ]
        tentabot_drl = roslaunch.parent.ROSLaunchParent(uuid, tentabot_drl_launch)
        tentabot_drl.start()

        print("tentabot_framework_launch:: __main__ -> Launched Tentabot-DRL: " + mode + "!")

    while (not rospy.is_shutdown()):
        pass