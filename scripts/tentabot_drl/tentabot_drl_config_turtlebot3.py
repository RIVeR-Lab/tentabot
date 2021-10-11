#!/usr/bin/env python3

'''
LAST UPDATE: 2021.10.08

AUTHOR: Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

NUA TODO:
'''

import rospy
import csv
import numpy as np

'''
DESCRIPTION: TODO...
'''
def write_data(file, data):
    file_status = open(file, 'a')
    with file_status:
        write = csv.writer(file_status)
        write.writerows(data)
        print("config::write_data -> Data is written in " + str(file))

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
def get_training_param(initial_trained_model_path, param_name) -> str:

    log_file = initial_trained_model_path + 'training_log.csv'
    with open(log_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row[0] == param_name:
                return row[1]

'''
NUA TODO: This config class seems unnecessary. Include all parameters into the main config file called by the launch file.
DESCRIPTION: TODO...good params: 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
'''
class Config():

    def __init__(self, data_folder_path="", odom={}, goal={}):        

        ## General
        self.mode = rospy.get_param('mode', "")

        if self.mode == "training":
            
            self.world_frame_name = rospy.get_param('world_frame_name', "")
            self.max_episode_steps = rospy.get_param('max_episode_steps', "")
            self.mantissa_precision = rospy.get_param('mantissa_precision', "")

            ## Sensors
            self.laser_size_downsampled = rospy.get_param('laser_size_downsampled', 0)
            self.laser_normalize_flag = rospy.get_param('laser_normalize_flag', False)

            ## Robots
            self.velocity_control_msg = ""
            self.velocity_control_data_path = rospy.get_param('velocity_control_data_path', "")
            velocity_control_data_str = read_data(self.velocity_control_data_path + "velocity_control_data.csv")
            self.velocity_control_data = np.zeros(velocity_control_data_str.shape)

            for i, row in enumerate(velocity_control_data_str):
                for j, val in enumerate(row):
                    self.velocity_control_data[i][j] = float(val)

            self.min_lateral_speed = min(self.velocity_control_data[:,0])               # [m/s]
            self.max_lateral_speed = max(self.velocity_control_data[:,0])               # [m/s]
            self.init_lateral_speed = self.velocity_control_data[0,0]                   # [m/s]

            self.min_angular_speed = min(self.velocity_control_data[:,1])               # [rad/s]
            self.max_angular_speed = max(self.velocity_control_data[:,1])               # [rad/s]
            self.init_angular_speed = self.velocity_control_data[0,1]                   # [rad/s]

            ## Algorithm
            self.observation_space_type = rospy.get_param('observation_space_type', "")

            self.goal_close_threshold = rospy.get_param("goal_close_threshold", 0.0)
            self.obs_min_range = rospy.get_param('obs_min_range', 0.0)

            self.n_actions = len(self.velocity_control_data)
            self.n_observations = self.n_actions

            self.n_obs_stack = rospy.get_param("n_obs_stack", 0.0)
            self.n_skip_obs_stack = rospy.get_param("n_skip_obs_stack", 0.0)

            self.cnn_obs_shape = (1,-1)
            self.fc_obs_shape = (-1, )

            if  self.observation_space_type == "Tentabot_1DCNN_FC" or \
                self.observation_space_type == "Tentabot_2DCNN_FC" or \
                self.observation_space_type == "Tentabot_2DCNN":

                self.cit_flag = rospy.get_param("cit_flag", True)

                if self.cit_flag == False:
                    self.cnn_obs_shape = (-1,1)

            # Rewards
            self.reward_success = rospy.get_param('reward_success', 0.0)
            self.penalty_failure = rospy.get_param('penalty_failure', 0.0)
            self.safety_range_threshold = rospy.get_param('safety_range_threshold', 0.0)
            self.penalty_safety_scale = rospy.get_param('penalty_safety_scale', 0.0)
            self.reward_goal_scale = rospy.get_param('reward_goal_scale', 0.0)

            if data_folder_path:

                ## Write all parameters
                training_log_file = data_folder_path + "training_log.csv"

                training_log_data = []
                training_log_data.append(["mode", self.mode])
                training_log_data.append(["world_frame_name", self.world_frame_name])
                training_log_data.append(["max_episode_steps", self.max_episode_steps])
                training_log_data.append(["mantissa_precision", self.mantissa_precision])
                training_log_data.append(["laser_size_downsampled", self.laser_size_downsampled])
                training_log_data.append(["laser_normalize_flag", self.laser_normalize_flag])
                training_log_data.append(["velocity_control_data_path", self.velocity_control_data_path])
                training_log_data.append(["min_lateral_speed", self.min_lateral_speed])
                training_log_data.append(["max_lateral_speed", self.max_lateral_speed])
                training_log_data.append(["init_lateral_speed", self.init_lateral_speed])
                training_log_data.append(["min_angular_speed", self.min_angular_speed])
                training_log_data.append(["max_angular_speed", self.min_angular_speed])
                training_log_data.append(["init_angular_speed", self.init_angular_speed])
                training_log_data.append(["observation_space_type", self.observation_space_type])
                training_log_data.append(["goal_close_threshold", self.goal_close_threshold])
                training_log_data.append(["obs_min_range", self.obs_min_range])
                training_log_data.append(["n_actions", self.n_actions])
                training_log_data.append(["n_observations", self.n_observations])
                training_log_data.append(["n_obs_stack", self.n_obs_stack])
                training_log_data.append(["n_skip_obs_stack", self.n_skip_obs_stack])
                training_log_data.append(["cnn_obs_shape", self.cnn_obs_shape])
                training_log_data.append(["fc_obs_shape", self.fc_obs_shape])
                training_log_data.append(["reward_success", self.reward_success])
                training_log_data.append(["penalty_failure", self.penalty_failure])
                training_log_data.append(["safety_range_threshold", self.safety_range_threshold])
                training_log_data.append(["penalty_safety_scale", self.penalty_safety_scale])
                training_log_data.append(["reward_goal_scale", self.reward_goal_scale])

                if  self.observation_space_type == "Tentabot_1DCNN_FC" or \
                    self.observation_space_type == "Tentabot_2DCNN_FC" or \
                    self.observation_space_type == "Tentabot_2DCNN":

                    training_log_data.append(["cit_flag", self.cit_flag])

                write_data(training_log_file, training_log_data)

        elif self.mode == "testing":
            
            self.initial_trained_model_path = rospy.get_param('initial_trained_model_path', "")

            self.world_frame_name = get_training_param(self.initial_trained_model_path, "world_frame_name")
            self.max_episode_steps = int(get_training_param(self.initial_trained_model_path, "max_episode_steps"))
            self.mantissa_precision = int(get_training_param(self.initial_trained_model_path, "mantissa_precision"))

            ## Sensors
            self.laser_size_downsampled = int(get_training_param(self.initial_trained_model_path, "laser_size_downsampled"))

            if get_training_param(self.initial_trained_model_path, "laser_normalize_flag") == "False":
                self.laser_normalize_flag = False
            else:
                self.laser_normalize_flag = True

            ## Robots
            self.velocity_control_msg = rospy.get_param('velocity_control_msg', "")
            self.velocity_control_data_path = get_training_param(self.initial_trained_model_path, "velocity_control_data_path")
            velocity_control_data_str = read_data(self.velocity_control_data_path + "velocity_control_data.csv")
            self.velocity_control_data = np.zeros(velocity_control_data_str.shape)

            for i, row in enumerate(velocity_control_data_str):
                for j, val in enumerate(row):
                    self.velocity_control_data[i][j] = float(val)

            self.min_lateral_speed = min(self.velocity_control_data[:,0])               # [m/s]
            self.max_lateral_speed = max(self.velocity_control_data[:,0])               # [m/s]
            self.init_lateral_speed = self.velocity_control_data[0,0]                   # [m/s]

            self.min_angular_speed = min(self.velocity_control_data[:,1])               # [rad/s]
            self.max_angular_speed = max(self.velocity_control_data[:,1])               # [rad/s]
            self.init_angular_speed = self.velocity_control_data[0,1]                   # [rad/s]

            ## Algorithm
            self.observation_space_type = get_training_param(self.initial_trained_model_path, "observation_space_type")

            self.goal_close_threshold = float(get_training_param(self.initial_trained_model_path, "goal_close_threshold"))
            self.obs_min_range = float(get_training_param(self.initial_trained_model_path, "obs_min_range"))

            self.n_actions = len(self.velocity_control_data)
            self.n_observations = self.n_actions

            self.n_obs_stack = int(get_training_param(self.initial_trained_model_path, "n_obs_stack"))
            self.n_skip_obs_stack = int(get_training_param(self.initial_trained_model_path, "n_skip_obs_stack"))

            self.cnn_obs_shape = (1,-1)
            self.fc_obs_shape = (-1, )

            if  self.observation_space_type == "Tentabot_1DCNN_FC" or \
                self.observation_space_type == "Tentabot_2DCNN_FC" or \
                self.observation_space_type == "Tentabot_2DCNN":

                if get_training_param(self.initial_trained_model_path, "cit_flag") == "False":
                    self.cit_flag = False
                else:
                    self.cit_flag = True

                if self.cit_flag == False:
                    self.cnn_obs_shape = (-1,1)

            # Rewards
            self.reward_success = float(get_training_param(self.initial_trained_model_path, "reward_success"))
            self.penalty_failure = float(get_training_param(self.initial_trained_model_path, "penalty_failure"))
            self.safety_range_threshold = float(get_training_param(self.initial_trained_model_path, "safety_range_threshold"))
            self.penalty_safety_scale = float(get_training_param(self.initial_trained_model_path, "penalty_safety_scale"))
            self.reward_goal_scale = float(get_training_param(self.initial_trained_model_path, "reward_goal_scale"))

            if data_folder_path:

                ## Write all parameters
                testing_log_file = data_folder_path + "testing_input_log.csv"

                testing_log_data = []
                testing_log_data.append(["mode", self.mode])
                testing_log_data.append(["initial_trained_model_path", self.initial_trained_model_path])
                
                write_data(testing_log_file, testing_log_data)

        print("config::__init__ -> world_frame_name: " + str(self.world_frame_name))
        print("config::__init__ -> max_episode_steps: " + str(self.max_episode_steps))
        print("config::__init__ -> mantissa_precision: " + str(self.mantissa_precision))
        print("config::__init__ -> laser_size_downsampled: " + str(self.laser_size_downsampled))
        print("config::__init__ -> laser_normalize_flag: " + str(self.laser_normalize_flag))
        print("config::__init__ -> velocity_control_data_path: " + str(self.velocity_control_data_path))
        print("config::__init__ -> observation_space_type: " + str(self.observation_space_type))
        print("config::__init__ -> goal_close_threshold: " + str(self.goal_close_threshold))
        print("config::__init__ -> obs_min_range: " + str(self.obs_min_range))
        print("config::__init__ -> n_actions: " + str(self.n_actions))
        print("config::__init__ -> n_observations: " + str(self.n_observations))
        print("config::__init__ -> n_obs_stack: " + str(self.n_obs_stack))
        print("config::__init__ -> n_skip_obs_stack: " + str(self.n_skip_obs_stack))
        print("config::__init__ -> reward_success: " + str(self.reward_success))
        print("config::__init__ -> penalty_failure: " + str(self.penalty_failure))
        print("config::__init__ -> safety_range_threshold: " + str(self.safety_range_threshold))
        print("config::__init__ -> penalty_safety_scale: " + str(self.penalty_safety_scale))
        print("config::__init__ -> reward_goal_scale: " + str(self.reward_goal_scale))

        # NUA REWARDS:
        #self.value_step_min = self.reward_success / (self.max_episode_steps**2 + 3*self.max_episode_steps)
        #self.value_step_max = self.value_step_min * (self.max_episode_steps + 1)

        if odom:
            self.x = odom["x"]
            self.y = odom["y"]
            self.th = odom["theta"]
            self.v = odom["u"]
            self.omega = odom["omega"]

        if goal:
            self.goalX = goal["x"]
            self.goalY = goal["y"]

        '''
        print("--------------")
        print("Config::__init__ -> x: " + str(odom["x"]))
        print("Config::__init__ -> y: " + str(odom["y"]))
        print("Config::__init__ -> theta: " + str(odom["theta"]))
        print("Config::__init__ -> u: " + str(odom["u"]))
        print("Config::__init__ -> omega: " + str(odom["omega"]))
        print("--------------")
        '''

    '''
    NUA TODO: 
    '''
    def set_odom(self, odom):

        self.x = odom["x"]
        self.y = odom["y"]
        self.theta = odom["theta"]
        self.v = odom["u"]
        self.omega = odom["omega"]

    '''
    NUA TODO: 
    '''
    def set_goal(self, goal):

        self.goalX = goal["x"]
        self.goalY = goal["y"]

    '''
    NUA TODO: 
    '''
    def set_laser_data(self, laser_scan):

        self.laser_frame_id = laser_scan.header.frame_id
        self.laser_angle_min = laser_scan.angle_min                   # [rad]
        self.laser_angle_max = laser_scan.angle_max                   # [rad]
        self.laser_angle_increment = laser_scan.angle_increment       # [rad]
        self.laser_range_min = laser_scan.range_min                   # [m]
        self.laser_range_max = laser_scan.range_max                   # [m]
        self.laser_time_increment = laser_scan.time_increment
        self.laser_scan_time = laser_scan.scan_time
        self.laser_n_range = len(laser_scan.ranges)
        self.laser_downsample_scale = 1
        if 0 < self.laser_size_downsampled < len(laser_scan.ranges):
            self.laser_downsample_scale = int(len(laser_scan.ranges) / self.laser_size_downsampled)
            self.laser_n_range = self.laser_size_downsampled
