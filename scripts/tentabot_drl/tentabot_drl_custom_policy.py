#!/usr/bin/env python3

'''
LAST UPDATE: 2022.02.23

AUTHOR:     OPENAI_ROS
            Neset Unver Akmandor (NUA)

E-MAIL: akmandor.n@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:
[1] 

NUA TODO:
'''

from typing import Callable, Dict, List, Optional, Tuple, Type, Union

import math
import gym
import numpy as np
import torch as th
import torch.nn as nn

from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.vec_env import VecTransposeImage
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.preprocessing import get_flattened_obs_dim

"""
NUA TODO...
"""
class laser_image_2DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("laser_image_2DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(laser_image_2DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("laser_image_2DCNN_FC_Policy::__init__ -> observation_space.spaces laser_image shape: " + str(observation_space.spaces["laser_image"].shape))
        print("laser_image_2DCNN_FC_Policy::__init__ -> observation_space.spaces target_action shape: " + str(observation_space.spaces["target_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["laser_image"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["laser_image"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["target_action"].shape[0]

        print("laser_image_2DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("laser_image_2DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("laser_image_2DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 64

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 64

        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("laser_image_2DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        
        self.cnn_net = nn.Sequential(

            nn.Conv2d(n_channel_input1, n_channel_output1, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(n_channel_input2, n_channel_output2, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(n_channel_input3, n_channel_output3, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "laser_image":
                extractors[key] = self.cnn_net
            
            elif key == "target_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["laser_image"].sample()[None]

            print("laser_image_2DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("laser_image_2DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("laser_image_2DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)

"""
NUA TODO...
"""
class laser_rings_2DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("laser_rings_2DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(laser_rings_2DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("laser_rings_2DCNN_FC_Policy::__init__ -> observation_space.spaces laser_image shape: " + str(observation_space.spaces["laser_image"].shape))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> observation_space.spaces target_action shape: " + str(observation_space.spaces["target_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["laser_image"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["laser_image"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["target_action"].shape[0]

        print("laser_rings_2DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 64

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 64

        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("laser_rings_2DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        
        self.cnn_net = nn.Sequential(

            nn.Conv2d(n_channel_input1, n_channel_output1, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(n_channel_input2, n_channel_output2, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(n_channel_input3, n_channel_output3, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "laser_image":
                extractors[key] = self.cnn_net
            
            elif key == "target_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["laser_image"].sample()[None]

            print("laser_rings_2DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("laser_rings_2DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("laser_rings_2DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)

"""
NUA TODO...
"""
class laser_1DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("laser_1DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(laser_1DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("laser_1DCNN_FC_Policy::__init__ -> observation_space.spaces laser shape: " + str(observation_space.spaces["laser"].shape))
        print("laser_1DCNN_FC_Policy::__init__ -> observation_space.spaces target_action shape: " + str(observation_space.spaces["target_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["laser"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["laser"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["target_action"].shape[0]

        print("laser_1DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("laser_1DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("laser_1DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 32

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 32

        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("laser_1DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        
        self.cnn_net = nn.Sequential(

            nn.Conv1d(n_channel_input1, n_channel_output1, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input2, n_channel_output2, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input3, n_channel_output3, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "laser":
                extractors[key] = self.cnn_net
            
            elif key == "target_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["laser"].sample()[None]

            print("laser_1DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("laser_1DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("laser_1DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)

"""
NUA TODO...
"""
class Tentabot_1DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("Tentabot_1DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(Tentabot_1DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("Tentabot_1DCNN_FC_Policy::__init__ -> observation_space.spaces occupancy shape: " + str(observation_space.spaces["occupancy"].shape))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> observation_space.spaces target_action shape: " + str(observation_space.spaces["target_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["occupancy"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["occupancy"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["target_action"].shape[0]

        print("Tentabot_1DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 32

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 32

        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("Tentabot_1DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        
        self.cnn_net = nn.Sequential(

            nn.Conv1d(n_channel_input1, n_channel_output1, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input2, n_channel_output2, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input3, n_channel_output3, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "occupancy":
                extractors[key] = self.cnn_net
            
            elif key == "target_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["occupancy"].sample()[None]

            print("Tentabot_1DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("Tentabot_1DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("Tentabot_1DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)

"""
NUA TODO...
"""
class Tentabot_2DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("Tentabot_2DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(Tentabot_2DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("Tentabot_2DCNN_FC_Policy::__init__ -> observation_space.spaces occupancy shape: " + str(observation_space.spaces["occupancy"].shape))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> observation_space.spaces target_action shape: " + str(observation_space.spaces["target_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["occupancy"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["occupancy"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["target_action"].shape[0]

        print("Tentabot_2DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        '''
        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 2 * n_channel_input1
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = n_channel_input2 + 2

        n_channel_input3 = n_channel_output2
        n_channel_output3 = n_channel_input3 + 2

        n_channel_input4 = n_channel_output3
        n_channel_output4 = n_channel_input4 + 2

        n_channel_input5 = n_channel_output4
        n_channel_output5 = n_channel_input5 + 2
        '''

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 16

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 8

        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        #print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_input4: " + str(n_channel_input4))
        #print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_output4: " + str(n_channel_output4))
        #print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_input5: " + str(n_channel_input5))
        #print("Tentabot_2DCNN_FC_Policy::__init__ -> n_channel_output5: " + str(n_channel_output5))
        
        self.cnn_net = nn.Sequential(

            nn.Conv2d(n_channel_input1, n_channel_output1, kernel_size=2, stride=1),
            nn.ReLU(),
            nn.Conv2d(n_channel_input2, n_channel_output2, kernel_size=2, stride=1),
            nn.ReLU(),
            nn.Conv2d(n_channel_input3, n_channel_output3, kernel_size=2, stride=1),
            nn.ReLU(),
            #nn.Conv2d(n_channel_input4, n_channel_output4, kernel_size=3, stride=1),
            #nn.ReLU(),
            #nn.Conv2d(n_channel_input5, n_channel_output5, kernel_size=3, stride=1),
            #nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "occupancy":
                extractors[key] = self.cnn_net
            
            elif key == "target_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["occupancy"].sample()[None]

            print("Tentabot_2DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("Tentabot_2DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("Tentabot_2DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)

class laserWayPoints_1DCNN_FC_Policy(BaseFeaturesExtractor):

    def __init__(self, observation_space: gym.spaces.Dict):
        # We do not know features-dim here before going over all the items,
        # so put something dummy for now. PyTorch requires calling
        # nn.Module.__init__ before adding modules

        print("laser_WP_1DCNN_FC_Policy::__init__ -> observation_space: " + str(observation_space))

        super(laserWayPoints_1DCNN_FC_Policy, self).__init__(observation_space, features_dim=1)

        print("laser_WP_1DCNN_FC_Policy::__init__ -> observation_space.spaces laser shape: " + str(observation_space.spaces["laser"].shape))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> observation_space.spaces waypoints_action shape: " + str(observation_space.spaces["waypoints_action"].shape))

        self.cnn_input_channel_len = observation_space.spaces["laser"].shape[0]
        self.cnn_input_data_len = observation_space.spaces["laser"].shape[1]
        self.fc_input_extra_data_len = observation_space.spaces["waypoints_action"].shape[0]

        print("laser_WP_1DCNN_FC_Policy::__init__ -> cnn_input_channel_len: " + str(self.cnn_input_channel_len))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> cnn_input_data_len: " + str(self.cnn_input_data_len))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> fc_input_extra_data_len: " + str(self.fc_input_extra_data_len))

        n_channel_input1 = self.cnn_input_channel_len
        n_channel_output1 = 32
        
        n_channel_input2 = n_channel_output1
        n_channel_output2 = 32

        n_channel_input3 = n_channel_output2
        n_channel_output3 = 32

        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_input1: " + str(n_channel_input1))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_output1: " + str(n_channel_output1))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_input2: " + str(n_channel_input2))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_output2: " + str(n_channel_output2))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_input3: " + str(n_channel_input3))
        print("laser_WP_1DCNN_FC_Policy::__init__ -> n_channel_output3: " + str(n_channel_output3))
        
        self.cnn_net = nn.Sequential(

            nn.Conv1d(n_channel_input1, n_channel_output1, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input2, n_channel_output2, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Conv1d(n_channel_input3, n_channel_output3, kernel_size=2, stride=2),
            nn.ReLU(),
            nn.Flatten(),
        )

        extractors = {}

        # We need to know size of the output of this extractor,
        # so go over all the spaces and compute output feature sizes
        for key, subspace in observation_space.spaces.items():
            
            if key == "laser":
                extractors[key] = self.cnn_net
            
            elif key == "waypoints_action":
                extractors[key] = nn.Flatten()

        # Compute shape by doing one forward pass
        with th.no_grad():

            cnn_input_sample = observation_space.spaces["laser"].sample()[None]

            print("laser_WP_1DCNN_FC_Policy::__init__ -> cnn_input_sample shape: " + str(cnn_input_sample.shape))

            cnn_output = self.cnn_net(th.as_tensor(cnn_input_sample).float())

            print("laser_WP_1DCNN_FC_Policy::__init__ -> cnn_output shape: " + str(cnn_output.shape))

            n_flatten = cnn_output.shape[1] + self.fc_input_extra_data_len

            print("laser_WP_1DCNN_FC_Policy::__init__ -> n_flatten: " + str(n_flatten))

        self.extractors = nn.ModuleDict(extractors)

        # Update the features dim manually
        self._features_dim = n_flatten

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))
        # Return a (B, self._features_dim) PyTorch tensor, where B is batch dimension.
        return th.cat(encoded_tensor_list, dim=1)