#!/usr/bin/env python3

'''
LAST UPDATE: 2022.04.03

AUTHOR: Neset Unver Akmandor (NUA)
        Eric Dusel (ED)

E-MAIL: akmandor.n@northeastern.edu
        dusel.e@nortehastern.edu

DESCRIPTION: TODO...

REFERENCES:
[1] 

NUA TODO:
- 
'''

import rospy
import rospkg
import os
import csv
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import time
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO

'''
DESCRIPTION: TODO...
'''
def createFileName():
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%Y%m%d_%H%M%S")
    return timestampStr

    return total_val / counter

'''
DESCRIPTION: TODO...
'''
def print_testing_result_log(log_path):

    with open(log_path + 'testing_result_log.csv') as csv_file:
        
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0

        print("----------------")
        print("testing_dwarl::print_testing_result_log ->")
        for row in csv_reader:
            print(str(line_count) + " -> " + str(row[0]) + ", " + str(row[1]) + ", " + str(row[2]))
            line_count += 1
        print("----------------")

'''
DESCRIPTION: TODO...
'''
def get_training_param(initial_training_path, param_name) -> str:

    log_file = initial_training_path + 'training_log.csv'
    with open(log_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] == param_name:
                return row[1]

'''
DESCRIPTION: TODO...
'''
def get_color_array(success_data):
    color_array = []
    for d in success_data:
        if d == 0.0:
            color_array.append("red")
        else:
            color_array.append("blue")
    return color_array

'''
DESCRIPTION: TODO...
'''
def get_successful_avg(data, success_data):
    total_val = 0.0;
    counter = 0

    for d in range(len(success_data)):
        if success_data[d] > 0:
            total_val += data[d]
        counter += 1
    return total_val / counter

'''
DESCRIPTION: TODO... plot the results
    :param log_path: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
'''
def plot_testing_result(log_path):

    success_data = []
    duration_data = []
    path_length_data = []

    with open(log_path + 'testing_result_log.csv') as csv_file:
        
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0

        for row in csv_reader:
            if line_count > 0:
                success_data.append(float(row[0]))
                duration_data.append(float(row[1]))
                path_length_data.append(float(row[2]))
            line_count += 1

    label_x = list(map(str, range(1,len(duration_data)+1,1)))
    color_array = get_color_array(success_data)
    
    avg_success = sum(success_data) / len(success_data)
    plt.figure(1)
    plt.bar(label_x, success_data, color=color_array)
    plt.axhline(avg_success, color='blue', linewidth=2)
    plt.xlabel('Episode Index')
    plt.ylabel('Navigation Success')
    plt.title("Navigation Success")
    plt.savefig(log_path + '/nav_success.png')

    avg_duration = get_successful_avg(duration_data, success_data)
    plt.figure(2)
    plt.bar(label_x, duration_data, color=color_array)
    plt.axhline(avg_duration, color='blue', linewidth=2)
    plt.xlabel('Episode Index')
    plt.ylabel('Navigation Duration [s]')
    plt.title("Navigation Duration")
    plt.savefig(log_path + '/nav_duration.png')

    avg_path_length = get_successful_avg(path_length_data, success_data)
    plt.figure(3)
    plt.bar(label_x, path_length_data, color=color_array)
    plt.axhline(avg_path_length, color='blue', linewidth=2)
    plt.xlabel('Episode Index')
    plt.ylabel('Navigation Path Length [m]')
    plt.title("Navigation Path Length")
    plt.savefig(log_path + '/nav_path_length.png')

'''
DESCRIPTION: TODO...
'''
def euclidean_distance(pos1, pos2):
    return np.sqrt( pow((pos1.x - pos2.x), 2) + pow((pos1.y - pos2.y), 2) + pow((pos1.z - pos2.z), 2))

'''
DESCRIPTION: TODO...
'''
prev_pos = Point()
total_distance_episode = 0.0
def getDistance(msg):
    global prev_pos, total_distance_episode
    dist = euclidean_distance(msg.pose.pose.position, prev_pos)
    total_distance_episode += dist
    prev_pos = msg.pose.pose.position

'''
DESCRIPTION: TODO...
'''
gr = 0.0
def getGoalReachingStatus(msg):
    global gr
    if msg.data:
        gr = 1.0
    else:
        gr = 0.0

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    rospy.init_node('tentabot_drl_testing', anonymous=True, log_level=rospy.WARN)

    rospack = rospkg.RosPack()
    tentabot_path = rospack.get_path('tentabot') + "/"

    mode = "testing"
    data_path = rospy.get_param('drl_data_path', "")
    initial_training_path = rospy.get_param('initial_training_path', "")
    world_name = rospy.get_param('world_name', "")
    max_testing_episode_timesteps = rospy.get_param('max_testing_episode_timesteps', 0)
    max_testing_episodes = rospy.get_param('max_testing_episodes', 0)
    velocity_control_msg = rospy.get_param('robot_velo_control_msg', "")
    odometry_msg = rospy.get_param('robot_odometry_msg', "")
    goal_status_msg = rospy.get_param('goal_status_msg', "")

    deep_learning_algorithm = get_training_param(tentabot_path + initial_training_path, "deep_learning_algorithm")
    task_and_robot_environment_name = get_training_param(tentabot_path + initial_training_path, "task_and_robot_environment_name")

    ## Create the folder name that the data is kept
    data_file_tag = createFileName()
    data_folder_tag = data_file_tag + "_" + deep_learning_algorithm + "_tentabot"
    data_path_specific = tentabot_path + data_path
    data_folder_path = data_path_specific + data_file_tag + "/"
    tensorboard_log_path = data_folder_path + deep_learning_algorithm + "_tensorboard/"

    os.makedirs(data_folder_path, exist_ok=True)

    ## Write all parameters and results into the log file of the training
    testing_input_log_data = []
    testing_input_log_data.append(["mode", mode])
    testing_input_log_data.append(["data_path", data_path])
    testing_input_log_data.append(["initial_training_path", initial_training_path])
    testing_input_log_data.append(["world_name", world_name])
    testing_input_log_data.append(["max_testing_episode_timesteps", max_testing_episode_timesteps])
    testing_input_log_data.append(["max_testing_episodes", max_testing_episodes])
    testing_input_log_data.append(["velocity_control_msg", velocity_control_msg])
    testing_input_log_data.append(["odometry_msg", odometry_msg])
    testing_input_log_data.append(["goal_status_msg", goal_status_msg])

    '''
    print("tentabot_drl_testing::__main__ -> mode: " + str(mode))
    print("tentabot_drl_testing::__main__ -> data_path: " + str(data_path))
    print("tentabot_drl_testing::__main__ -> initial_training_path: " + str(initial_training_path))
    print("tentabot_drl_testing::__main__ -> world_name: " + str(world_name))
    print("tentabot_drl_testing::__main__ -> max_testing_episode_timesteps: " + str(mode))
    print("tentabot_drl_testing::__main__ -> max_testing_episodes: " + str(max_testing_episodes))
    print("tentabot_drl_testing::__main__ -> velocity_control_msg: " + str(velocity_control_msg))
    print("tentabot_drl_testing::__main__ -> odometry_msg: " + str(odometry_msg))
    print("tentabot_drl_testing::__main__ -> goal_status_msg: " + str(goal_status_msg))
    print("tentabot_drl_testing::__main__ -> data_folder_path: " + str(data_folder_path))
    print("tentabot_drl_testing::__main__ -> deep_learning_algorithm: " + str(deep_learning_algorithm))
    print("tentabot_drl_testing::__main__ -> task_and_robot_environment_name: " + str(task_and_robot_environment_name))
    '''
    
    testing_input_log_file = data_folder_path + "testing_input_log.csv"
    input_file = open(testing_input_log_file, 'w')
    with input_file:
        write = csv.writer(input_file)
        write.writerows(testing_input_log_data)

    # Subscribers
    prev_pos = rospy.wait_for_message(odometry_msg, Odometry, timeout=1.0).pose.pose.position
    totalDistance = rospy.Subscriber(odometry_msg, Odometry, getDistance)
    goal_reaching_status = rospy.Subscriber(goal_status_msg, Bool, getGoalReachingStatus)

    # Set the environment
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name, robot_id=0, max_episode_steps=max_testing_episode_timesteps, data_folder_path=data_folder_path)
    env = Monitor(env, data_folder_path)

    # Initialize parameters
    counter = 0
    start_time = rospy.get_time()
    episode_result_list = []
    goal_reached = 0.0
    initial_trained_model = tentabot_path + initial_training_path + "trained_model"

    if world_name == "validation":
        max_testing_episodes *= 5

    while(counter < max_testing_episodes):

        model = PPO.load(initial_trained_model, env=None, tensorboard_log=tensorboard_log_path)
        print("--------------")
        print("tentabot_drl_testing::__main__ -> Testing episode " + str(counter) + ": Loaded initial_trained_model: " + initial_trained_model)
        print("--------------")

        model.set_env(env)
        obs = env.reset()

        # Evaluate the agent
        episode_reward = 0
        total_distance_episode = 0.0
        start_time = rospy.get_time()

        # if goal_reached == 1.0:
        #     break

        for i in range(max_testing_episode_timesteps):

            #print("--------------")
            #print("tentabot_drl_testing::__main__ -> i: {}".format(i))
            #print("--------------")

            action, _ = model.predict(obs)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
            goal_reached = gr
            
            if (done):
                #print("--------------")
                #print("tentabot_drl_testing::__main__ -> Done!")
                #print("--------------")
                counter += 1

                total_time_episode = rospy.get_time() - start_time
                    
                print("--------------")
                print("tentabot_drl_testing::__main__ -> goal_reached: {}".format(goal_reached))
                print("tentabot_drl_testing::__main__ -> total_time_episode: {}".format(total_time_episode))
                print("tentabot_drl_testing::__main__ -> total_distance_episode: {}".format(total_distance_episode))
                print("--------------")

                episode_result_list.append([goal_reached, total_time_episode, total_distance_episode])
                break
            
            else:
                if i == max_testing_episode_timesteps-1:
                    print("--------------")
                    print("tentabot_drl_testing::__main__ -> Max number of timesteps has been reached!")
                    print("--------------")
                    counter += 1

                    total_time_episode = rospy.get_time() - start_time
                    
                    print("--------------")
                    print("tentabot_drl_testing::__main__ -> goal_reached: {}".format(goal_reached))
                    print("tentabot_drl_testing::__main__ -> total_time_episode: {}".format(total_time_episode))
                    print("tentabot_drl_testing::__main__ -> total_distance_episode: {}".format(total_distance_episode))
                    print("--------------")

                    episode_result_list.append([goal_reached, total_time_episode, total_distance_episode])
                    
                    obs = env.reset()
                    break

    ## Write the navigation results per episode into the log file of the testing
    testing_result_log_file = data_folder_path + "testing_result_log.csv"
    result_file = open(testing_result_log_file, 'w')
    with result_file:     
        write = csv.writer(result_file)
        episode_result_list.insert(0, ["success", "duration", "path_length"])
        write.writerows(episode_result_list)

    ## Save the result plots of the testing
    plot_testing_result(data_folder_path)