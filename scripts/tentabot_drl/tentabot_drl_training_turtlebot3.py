#!/usr/bin/env python3

'''
LAST UPDATE: 2021.10.08

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

import sys
import gym
import rospy
import rospkg
import os
import csv
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
import time
from collections import deque
import os.path as osp
import json

from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from stable_baselines3.common.vec_env import SubprocVecEnv, VecEnvWrapper
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env

from tentabot_drl_custom_policy import *

'''
DESCRIPTION: TODO...
'''
def createFileName():
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%Y%m%d_%H%M%S")
    return timestampStr

'''
DESCRIPTION: TODO...
'''
def read_data(file):
    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = np.array(next(reader))
        for row in reader:
            data_row = np.array(row)
            data = np.vstack((data, data_row))
        return data
        
'''
DESCRIPTION: TODO...
'''
def write_data(file, data):
    file_status = open(file, 'a')
    with file_status:
        write = csv.writer(file_status)
        write.writerows(data)
        print("training_tentabot_rl::write_data -> Data is written in " + str(file))

'''
DESCRIPTION: TODO...
'''
def print_array(arr):
    for i in range(len(arr)):
        print(str(i) + " -> " + str(arr[i]))

'''
DESCRIPTION: TODO...
'''
def print_training_log(log_path):

    with open(log_path + 'training_log.csv') as csv_file:
        
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0

        for row in csv_reader:
            print("training_tentabot_rl::print_training_log -> Line " + str(line_count) + " -> " + str(row[0]) + ": " + str(row[1]))
            line_count += 1

'''
DESCRIPTION: TODO...
'''
def get_param_value_from_training_log(log_path, param_name):

    with open(log_path + 'training_log.csv') as csv_file:
        
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if row[0] == param_name:
                return row[1]

'''
DESCRIPTION: TODO...Smooth values by doing a moving average
    :param values: (numpy array)
    :param window: (int)
    :return: (numpy array)
'''
def moving_average(values, window):

    weights = np.repeat(1.0, window) / window
    return np.convolve(values, weights, 'valid')

'''
DESCRIPTION: TODO...
'''
def plot_training_result(log_path, title='Learning Curve', moving_average_window_size=5, print_data=False):
    """
    plot the results

    :param log_path: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
    """
    x, y = ts2xy(load_results(log_path), 'timesteps')

    y = moving_average(y, window=moving_average_window_size)
    # Truncate x
    x = x[len(x) - len(y):]

    if print_data:
        print("----------")
        print("x:")
        print_array(x)
        print("----------")
        print("y:")
        print_array(y)
        print("----------")
        print("y_avg: " + str(np.mean(y)))
        print("----------")

    plt.figure()
    plt.plot(x, y)
    plt.xlabel('Number of Timesteps')
    plt.ylabel('Rewards')
    plt.title(title + " (smoothed)")
    plt.savefig(log_path + '/training_result.png')


'''
DESCRIPTION: TODO...
'''
def plot_cumulative_training_result(log_path, title='Learning Curve', moving_average_window_size=5, print_data=False):
    """
    plot the results

    :param log_path: (str) the save location of the results to plot
    :param title: (str) the title of the task to plot
    """

    current_path = log_path
    x_tot = np.empty(0)
    y_tot = np.empty(0)
    step_num_data = np.empty(0)
    step_add_data = np.empty(0)

    while(current_path != ""):

        x, y = ts2xy(load_results(current_path), 'timesteps')

        step_num_data = np.insert(step_num_data, 0, int(len(x)), axis=0)
        step_add_data = np.insert(step_add_data, 0, int(get_param_value_from_training_log(current_path, "total_training_timesteps")), axis=0)

        x_tot = np.insert(x_tot, 0, x, axis=0)
        y_tot = np.insert(y_tot, 0, y, axis=0)

        current_path = get_param_value_from_training_log(current_path, "initial_trained_model_path")

    y_tot = moving_average(y_tot, window=moving_average_window_size)

    s = 0
    e = int(step_num_data[0])
    add_step = 0
    
    for i in range(len(step_num_data)): 
        if i > 0:
            s = e
            e = s + int(step_num_data[i])
            add_step = step_add_data[i-1]

        x_tot[s:e] = x_tot[s:e] + add_step

    # Truncate x
    x_tot = x_tot[len(x_tot) - len(y_tot):]

    if print_data:
        print("x_tot:")
        print_array(x_tot)
        print("----------")
        print("y_tot:")
        print("----------")
        print_array(y_tot)
        print("----------")
        print("y_tot_avg: " + str(np.mean(y_tot)))
        print("----------")

    plt.figure()
    plt.plot(x_tot, y_tot)
    plt.xlabel('Number of Timesteps')
    plt.ylabel('Rewards')
    plt.title("Cumulative " + title + " (smoothed)")
    plt.savefig(log_path + '/cumulative_training_result.png')

'''
DESCRIPTION: TODO...
'''
def createFileName():
    dateTimeObj = datetime.now()
    timestampStr = dateTimeObj.strftime("%Y%m%d_%H%M%S")
    return timestampStr

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    rospy.init_node('training_tentabot_drl', anonymous=True, log_level=rospy.WARN)

    ## Initialize the parameters
    mode = rospy.get_param('mode', "")
    deep_learning_algorithm = rospy.get_param('deep_learning_algorithm', "")
    motion_planning_algorithm = rospy.get_param('motion_planning_algorithm', "")
    observation_space_type = rospy.get_param('observation_space_type', "")
    world_name = rospy.get_param('world_name', "")
    task_and_robot_environment_name = rospy.get_param('task_and_robot_environment_name', "")
    number_of_robots = rospy.get_param('number_of_robots', 0)
    data_path = rospy.get_param('data_path', "")
    learning_rate = rospy.get_param('learning_rate', 0)
    n_steps = rospy.get_param('n_steps', 0)
    batch_size = rospy.get_param('batch_size', 0)
    ent_coef = rospy.get_param('ent_coef', 0)
    training_timesteps = rospy.get_param('training_timesteps', 0)
    max_episode_steps = rospy.get_param('max_episode_steps', 0)
    initial_trained_model_path = rospy.get_param('initial_trained_model_path', "")
    plot_title = rospy.get_param('plot_title', "")
    plot_moving_average_window_size = rospy.get_param('plot_moving_average_window_size', 0)

    ## Create the folder name that the data is kept
    data_file_tag = createFileName()
    data_folder_tag = data_file_tag + "_" + deep_learning_algorithm + "_tentabot"
    data_folder_path = data_path + data_folder_tag + "/"

    os.makedirs(data_folder_path, exist_ok=True)

    new_trained_model_file = data_folder_path + "trained_model"
    training_log_file = data_folder_path + "training_log.csv"
    tensorboard_log_path = data_folder_path + deep_learning_algorithm + "_tensorboard/"

    ## Keep all parameters in an array to save
    training_log_data = []
    training_log_data.append(["mode", mode])
    training_log_data.append(["deep_learning_algorithm", deep_learning_algorithm])
    training_log_data.append(["motion_planning_algorithm", motion_planning_algorithm])
    training_log_data.append(["observation_space_type", observation_space_type])
    training_log_data.append(["world_name", world_name])
    training_log_data.append(["task_and_robot_environment_name", task_and_robot_environment_name])
    training_log_data.append(["number_of_robots", number_of_robots])
    training_log_data.append(["data_path", data_path])
    training_log_data.append(["learning_rate", learning_rate])
    training_log_data.append(["n_steps", n_steps])
    training_log_data.append(["batch_size", batch_size])
    training_log_data.append(["ent_coef", ent_coef])
    training_log_data.append(["training_timesteps", training_timesteps])
    training_log_data.append(["max_episode_steps", max_episode_steps])
    training_log_data.append(["initial_trained_model_path", initial_trained_model_path])
    training_log_data.append(["plot_title", plot_title])
    training_log_data.append(["plot_moving_average_window_size", plot_moving_average_window_size])

    ## Write all parameters into the log file of the training
    write_data(training_log_file, training_log_data)

    print("tentabot_drl_training_turtlebot3::__main__ -> task_and_robot_environment_name: " + str(task_and_robot_environment_name))
    print("tentabot_drl_training_turtlebot3::__main__ -> data_folder_path: " + str(data_folder_path))

    # Init OpenAI_ROS ENV
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name + str(0), robot_id=0, max_episode_steps=max_episode_steps, data_folder_path=data_folder_path)
    env = Monitor(env, data_folder_path)

    obs_space = env.observation_space

    print("training_tentabot_rl::__main__ -> shape_obs: " + str(obs_space))

    if observation_space_type == "lidar_FC" or observation_space_type == "Tentabot_FC":
        
        n_actions = env.action_space.n
        print("training_tentabot_rl::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(activation_fn=th.nn.ReLU, net_arch=[dict(pi=[400, 300], vf=[400, 300])])
        model = PPO("MlpPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "lidar_1DCNN_FC":
    
        print("training_tentabot_rl::__main__ -> observation_space_type: " + str(observation_space_type))
        n_actions = env.action_space.n
        #n_actions = env.action_space.shape[-1]
        print("training_tentabot_rl::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=lidar_1DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "Tentabot_1DCNN_FC":
    
        n_actions = env.action_space.n
        print("training_tentabot_rl::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=Tentabot_1DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "Tentabot_2DCNN_FC":
    
        n_actions = env.action_space.n
        print("training_tentabot_rl::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=Tentabot_2DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    

    if initial_trained_model_path == "":
        total_training_timesteps = training_timesteps
        print("--------------")
        print("training_tentabot_rl::__main__ -> No initial_trained_model is loaded!")
        print("--------------")
        rospy.logdebug("training_tentabot_rl::__main__ -> No initial_trained_model is loaded!")

    else:
        initial_trained_model = initial_trained_model_path + "trained_model"
        model = PPO.load(initial_trained_model, env=None, tensorboard_log=tensorboard_log_path)
        model.set_env(env)
        total_training_timesteps = int(get_param_value_from_training_log(initial_trained_model_path, "total_training_timesteps")) + training_timesteps
        print("--------------")
        print("training_tentabot_rl::__main__ -> Loaded initial_trained_model: " + initial_trained_model)
        print("--------------")
        #rospy.logdebug("training_tentabot_rl::__main__ -> Loaded initial_trained_model: " + initial_trained_model)
    
    start_learning = time.time()
    model.learn(total_timesteps=training_timesteps)
    end_learning = time.time()
    
    model.save(new_trained_model_file)

    learning_time = (end_learning - start_learning) / 60

    training_log_data = []
    training_log_data.append(["total_training_timesteps", total_training_timesteps])
    training_log_data.append(["learning_time[min]", learning_time])

    print("--------------")
    print("training_tentabot_rl::__main__ -> End of training!")
    print("training_tentabot_rl::__main__ -> learning_time[min]: " + str(learning_time))
    print("--------------")
    #rospy.logdebug("training_tentabot_rl::__main__ -> End of training!")

    ## Write all results into the log file of the training
    write_data(training_log_file, training_log_data)

    ## Save the reward plot
    plot_training_result(data_folder_path, plot_title, plot_moving_average_window_size)
    plot_cumulative_training_result(data_folder_path, plot_title, plot_moving_average_window_size)