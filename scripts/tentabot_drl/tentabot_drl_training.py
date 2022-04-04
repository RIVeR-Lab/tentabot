#!/usr/bin/env python3

'''
LAST UPDATE: 2022.04.03

AUTHOR: Neset Unver Akmandor (NUA)
        Eric Dusel (ED)
        Hongyu Li (LHY)

E-MAIL: akmandor.n@northeastern.edu
        dusel.e@northeastern.edu
        li.hongyu1@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

import rospy
import rospkg
import os
import csv
from datetime import datetime
import time

from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment

from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback

from tentabot_drl_custom_policy import *
from tentabot_drl.tentabot_drl_plot_result_training import get_training_result, read_data_size

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
        print("tentabot_drl_training::write_data -> Data is written in " + str(file))

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
            print("tentabot_drl_training::print_training_log -> Line " + str(line_count) + " -> " + str(row[0]) + ": " + str(row[1]))
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
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    rospy.init_node('tentabot_drl_training', anonymous=True, log_level=rospy.WARN)

    rospack = rospkg.RosPack()
    tentabot_path = rospack.get_path('tentabot') + "/"

    ## Initialize the parameters
    mode = rospy.get_param('mode', "")
    deep_learning_algorithm = rospy.get_param('deep_learning_algorithm', "")
    motion_planning_algorithm = rospy.get_param('motion_planning_algorithm', "")
    observation_space_type = rospy.get_param('observation_space_type', "")
    world_name = rospy.get_param('world_name', "")
    task_and_robot_environment_name = rospy.get_param('task_and_robot_environment_name', "")
    n_robot = rospy.get_param('n_robot', 0)
    data_path = rospy.get_param('drl_data_path', "")
    learning_rate = rospy.get_param('learning_rate', 0)
    n_steps = rospy.get_param('n_steps', 0)
    batch_size = rospy.get_param('batch_size', 0)
    ent_coef = rospy.get_param('ent_coef', 0)
    training_timesteps = rospy.get_param('training_timesteps', 0)
    max_episode_steps = rospy.get_param('max_episode_steps', 0)
    initial_training_path = rospy.get_param('initial_training_path', "")
    training_checkpoint_freq = rospy.get_param('training_checkpoint_freq', 0)
    plot_title = rospy.get_param('plot_title', "")
    plot_moving_average_window_size_timesteps = rospy.get_param('plot_moving_average_window_size_timesteps', 0)
    plot_moving_average_window_size_episodes = rospy.get_param('plot_moving_average_window_size_episodes', 0)

    '''
    print("tentabot_drl_training::__main__ -> mode: " + str(mode))
    print("tentabot_drl_training::__main__ -> deep_learning_algorithm: " + str(deep_learning_algorithm))
    print("tentabot_drl_training::__main__ -> motion_planning_algorithm: " + str(motion_planning_algorithm))
    print("tentabot_drl_training::__main__ -> observation_space_type: " + str(observation_space_type))
    print("tentabot_drl_training::__main__ -> world_name: " + str(world_name))
    print("tentabot_drl_training::__main__ -> task_and_robot_environment_name: " + str(task_and_robot_environment_name))
    print("tentabot_drl_training::__main__ -> n_robot: " + str(n_robot))
    print("tentabot_drl_training::__main__ -> data_path: " + str(data_path))
    print("tentabot_drl_training::__main__ -> learning_rate: " + str(learning_rate))
    print("tentabot_drl_training::__main__ -> n_steps: " + str(n_steps))
    print("tentabot_drl_training::__main__ -> batch_size: " + str(batch_size))
    print("tentabot_drl_training::__main__ -> ent_coef: " + str(ent_coef))
    print("tentabot_drl_training::__main__ -> training_timesteps: " + str(training_timesteps))
    print("tentabot_drl_training::__main__ -> max_episode_steps: " + str(max_episode_steps))
    print("tentabot_drl_training::__main__ -> initial_training_path: " + str(initial_training_path))
    print("tentabot_drl_training::__main__ -> training_checkpoint_freq: " + str(training_checkpoint_freq))
    print("tentabot_drl_training::__main__ -> plot_title: " + str(plot_title))
    print("tentabot_drl_training::__main__ -> plot_moving_average_window_size_timesteps: " + str(plot_moving_average_window_size_timesteps))
    print("tentabot_drl_training::__main__ -> plot_moving_average_window_size_episodes: " + str(plot_moving_average_window_size_episodes))
    '''
    
    ## Create the folder name that the data is kept
    data_file_tag = createFileName()
    data_folder_tag = data_file_tag + "_" + deep_learning_algorithm + "_tentabot"
    data_name = data_folder_tag + "/"
    data_path_specific = tentabot_path + data_path
    data_folder_path = data_path_specific + data_name

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
    training_log_data.append(["n_robot", n_robot])
    training_log_data.append(["data_path", data_path])
    training_log_data.append(["learning_rate", learning_rate])
    training_log_data.append(["n_steps", n_steps])
    training_log_data.append(["batch_size", batch_size])
    training_log_data.append(["ent_coef", ent_coef])
    training_log_data.append(["training_timesteps", training_timesteps])
    training_log_data.append(["max_episode_steps", max_episode_steps])
    training_log_data.append(["initial_training_path", initial_training_path])
    training_log_data.append(["training_checkpoint_freq", training_checkpoint_freq])
    training_log_data.append(["plot_title", plot_title])
    training_log_data.append(["plot_moving_average_window_size_timesteps", plot_moving_average_window_size_timesteps])
    training_log_data.append(["plot_moving_average_window_size_episodes", plot_moving_average_window_size_episodes])

    ## Write all parameters into the log file of the training
    write_data(training_log_file, training_log_data)

    # Init OpenAI_ROS ENV
    env = StartOpenAI_ROS_Environment(task_and_robot_environment_name, robot_id=0, max_episode_steps=max_episode_steps, data_folder_path=data_folder_path)
    env = Monitor(env, data_folder_path)

    #obs_space = env.observation_space
    #print("tentabot_drl_training::__main__ -> shape_obs: " + str(obs_space))

    if observation_space_type == "laser_FC" or \
        observation_space_type == "Tentabot_FC" or \
        observation_space_type == "Tentabot_WP_FC":
        
        n_actions = env.action_space.n
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(activation_fn=th.nn.ReLU, net_arch=[dict(pi=[400, 300], vf=[400, 300])])
        model = PPO("MlpPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "laser_image_2DCNN_FC":
    
        print("tentabot_drl_training::__main__ -> observation_space_type: " + str(observation_space_type))
        n_actions = env.action_space.n
        #n_actions = env.action_space.shape[-1]
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=laser_image_2DCNN_FC_Policy, net_arch=[dict(pi=[600, 400], vf=[600, 400])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "laser_rings_2DCNN_FC":
    
        print("tentabot_drl_training::__main__ -> observation_space_type: " + str(observation_space_type))
        n_actions = env.action_space.n
        #n_actions = env.action_space.shape[-1]
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=laser_rings_2DCNN_FC_Policy, net_arch=[dict(pi=[600, 400], vf=[600, 400])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "laser_1DCNN_FC":
    
        print("tentabot_drl_training::__main__ -> observation_space_type: " + str(observation_space_type))
        n_actions = env.action_space.n
        #n_actions = env.action_space.shape[-1]
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=laser_1DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "Tentabot_1DCNN_FC":
    
        n_actions = env.action_space.n
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=Tentabot_1DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "Tentabot_2DCNN_FC":
    
        n_actions = env.action_space.n
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=Tentabot_2DCNN_FC_Policy, net_arch=[dict(pi=[600, 400], vf=[600, 400])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    elif observation_space_type == "laser_WP_1DCNN_FC":

        print("tentabot_drl_training::__main__ -> observation_space_type: " + str(observation_space_type))
        n_actions = env.action_space.n
        #n_actions = env.action_space.shape[-1]
        print("tentabot_drl_training::__main__ -> n_actions: " + str(n_actions))

        policy_kwargs = dict(features_extractor_class=laserWayPoints_1DCNN_FC_Policy, net_arch=[dict(pi=[400, 300], vf=[400, 300])],)
        model = PPO("MultiInputPolicy", env, learning_rate=learning_rate, n_steps=n_steps, batch_size=batch_size, ent_coef=ent_coef, tensorboard_log=tensorboard_log_path, policy_kwargs=policy_kwargs, device="cuda", verbose=1)

    if initial_training_path == "":
        total_training_timesteps = training_timesteps
        print("--------------")
        print("tentabot_drl_training::__main__ -> No initial_trained_model is loaded!")
        print("--------------")
        rospy.logdebug("tentabot_drl_training::__main__ -> No initial_trained_model is loaded!")

    else:
        initial_training_path_specific = tentabot_path + data_path + initial_training_path
        initial_trained_model = initial_training_path + "trained_model"
        model = PPO.load(initial_trained_model, env=None, tensorboard_log=tensorboard_log_path)
        model.set_env(env)

        total_training_timesteps = int(get_param_value_from_training_log(initial_training_path_specific, "total_training_timesteps")) + training_timesteps
        print("--------------")
        print("tentabot_drl_training::__main__ -> Loaded initial_trained_model: " + initial_trained_model)
        print("--------------")
        #rospy.logdebug("tentabot_drl_training::__main__ -> Loaded initial_trained_model: " + initial_trained_model)

    checkpoint_callback = CheckpointCallback(save_freq=training_checkpoint_freq, save_path=data_folder_path + '/training_checkpoints/', name_prefix='trained_model')

    start_learning = time.time()
    model.learn(total_timesteps=training_timesteps, callback=checkpoint_callback)
    end_learning = time.time()
    
    model.save(new_trained_model_file)

    learning_time = (end_learning - start_learning) / 60

    total_training_episodes = read_data_size(data_folder_path + "training_data.csv")

    training_log_data = []
    training_log_data.append(["total_training_episodes", total_training_episodes])
    training_log_data.append(["total_training_timesteps", total_training_timesteps])
    training_log_data.append(["learning_time[min]", learning_time])

    print("--------------")
    print("tentabot_drl_training::__main__ -> End of training!")
    print("tentabot_drl_training::__main__ -> learning_time[min]: " + str(learning_time))
    print("--------------")
    #rospy.logdebug("tentabot_drl_training::__main__ -> End of training!")

    ## Write all results into the log file of the training
    write_data(training_log_file, training_log_data)

    ## Save the reward plot

    # Plot timestep data
    get_training_result(data_path=data_path_specific,
                        data_name=data_name,
                        moving_average_window_size=plot_moving_average_window_size_timesteps,
                        episode_flag=False,
                        cumulative_flag=False,
                        plot_title=plot_title, 
                        plot_flag=True)
    
    get_training_result(data_path=data_path_specific,
                        data_name=data_name,   
                        moving_average_window_size=plot_moving_average_window_size_timesteps,
                        episode_flag=False,
                        cumulative_flag=True,
                        plot_title=plot_title,
                        plot_flag=True)

    # Plot episode data
    get_training_result(data_path=data_path_specific,
                        data_name=data_name,  
                        moving_average_window_size=plot_moving_average_window_size_episodes,
                        episode_flag=True,
                        cumulative_flag=False,
                        plot_title=plot_title,
                        plot_flag=True)

    get_training_result(data_path=data_path_specific,
                        data_name=data_name,   
                        moving_average_window_size=plot_moving_average_window_size_episodes,
                        episode_flag=True,
                        cumulative_flag=True, 
                        plot_title=plot_title,
                        plot_flag=True)