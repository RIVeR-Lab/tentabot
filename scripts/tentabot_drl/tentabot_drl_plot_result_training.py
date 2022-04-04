#!/usr/bin/env python3

'''
LAST UPDATE: 2022.03.10

AUTHOR:	Neset Unver Akmandor (NUA)	
		Eric Dusel (ED)

E-MAIL: akmandor.n@northeastern.edu
		dusel.e@northeastern.edu

DESCRIPTION: TODO...

REFERENCES:

NUA TODO:
- 
'''

import sys
import rospy
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

from stable_baselines3.common.results_plotter import load_results, ts2xy

'''
DESCRIPTION: TODO...
'''
def read_data_size(file):
    with open(file, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        data = np.array(next(reader))
        i = 0
        for row in reader:
            i += 1
        return i

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

		print("----------")
		print("tentabot_drl_plot_result_training::print_training_log ->")
		for row in csv_reader:
			print(str(line_count) + " -> " + str(row[0]) + ": " + str(row[1]))
			line_count += 1
		print("----------")

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
DESCRIPTION: TODO...plot the results

	:param data_path: (str) the save location of the results to plot
	:param title: (str) the title of the task to plot
'''
def get_training_result(data_path,
						data_name,
						moving_average_window_size=5,
						episode_flag=False,
						cumulative_flag=False,
						print_flag=False,
						plot_title='Learning Curve', 
						plot_label="", 
						plot_flag=False, 
						plot_path="",
						plot_name=""):

	current_path = data_path + data_name

	x_tot = np.empty(0)
	y_tot = np.empty(0)
	step_num_data = np.empty(0)
	step_add_data = np.empty(0)
	x_label = 'Number of Timesteps'
	data_size_array = np.empty(0)

	while(current_path != ""):

		#print("")
		#print("get_training_result -> current_path: " + str(current_path))
		#print("")

		if episode_flag:

			y = []
			training_data_str = read_data(current_path + "training_data.csv")
			
			for i, row in enumerate(training_data_str):
				for j, val in enumerate(row):
					if i > 0:
						y.append(float(val))

			data_size = len(y)
			x = [*range(data_size)]
			x_label = 'Number of Episodes'

			if cumulative_flag and plot_name == "":
				plot_name = "cumulative_training_result_episodes.png"
			elif plot_name == "":
				plot_name = "training_result_episodes.png"

			data_size_array = np.insert(data_size_array, 0, data_size, axis=0)
			
		else:

			x, y = ts2xy(load_results(current_path), 'timesteps')
			
			data_size = int(get_param_value_from_training_log(current_path, "total_training_timesteps"))
			step_add_data = np.insert(step_add_data, 0, data_size, axis=0)

			if cumulative_flag and plot_name == "":
				plot_name = "cumulative_training_result_timesteps.png"

			elif plot_name == "":
				plot_name = "training_result_timesteps.png"

		step_num_data = np.insert(step_num_data, 0, int(len(x)), axis=0)
		
		x_tot = np.insert(x_tot, 0, x, axis=0)
		y_tot = np.insert(y_tot, 0, y, axis=0)

		initial_training_path = get_param_value_from_training_log(current_path, "initial_training_path")

		if cumulative_flag and initial_training_path != "":
			current_path = data_path + initial_training_path

		else:
			current_path = ""

	y_tot = moving_average(y_tot, window=moving_average_window_size)

	s = 0
	e = int(step_num_data[0])
	add_step = 0

	if episode_flag:
		for i in range(len(data_size_array)):
			step_add_data = np.append(step_add_data, sum(data_size_array[0:i+1]))

	for i in range(len(step_num_data)):	
		if i > 0:
			s = e
			e = s + int(step_num_data[i])
			add_step = step_add_data[i-1]
			x_tot[s:e] = x_tot[s:e] + add_step

	# Truncate x
	x_tot = x_tot[len(x_tot) - len(y_tot):]

	if print_flag:
		print("x_tot:")
		print_array(x_tot)
		print("----------")
		print("y_tot:")
		print("----------")
		print_array(y_tot)
		print("----------")
		print("y_tot_avg: " + str(np.mean(y_tot)))
		print("----------")

	if plot_flag:
		plt.figure()
		plt.plot(x_tot, y_tot, label=plot_label)
		plt.xlabel(x_label)
		plt.ylabel('Rewards')
		plt.title(plot_title + " (Avg. window size = " + str(moving_average_window_size) + ")")

		if plot_path:
			plt.savefig(plot_path + plot_name)
		else:
			plt.savefig(data_path + data_name + plot_name)

		plt.close()

	return [x_tot, y_tot]

'''
DESCRIPTION: TODO...
'''
def plot_multi_training_result(	data_path_array, 
								data_name_array, 
								plot_moving_average_window_size_timesteps=100,
								plot_moving_average_window_size_episodes=20,
								plot_title='Learning Curve',
								plot_flag=False, 
								plot_path=""):

	color_array = ['r', 'g', 'b', 'y', 'm', 'c', 'k']

	#plt.figure()
	plt.xlabel('Number of Timesteps')
	plt.ylabel('Rewards')
	plt.title(plot_title + " (Avg. window size = " + str(plot_moving_average_window_size_timesteps) + ")")

	for i, data_path in enumerate(data_path_array):

		data_name = data_name_array[i]
		current_path = data_path + data_name

		if data_name == "":
			
			data_name_array_auto = sorted(os.listdir(data_path_array[i]))

			for j, data_name_auto in enumerate(data_name_array_auto):
				
				data_name_auto = data_name_auto + "/"

				[x_tot, y_tot] = get_training_result(	data_path,
														data_name_auto,
														moving_average_window_size=plot_moving_average_window_size_timesteps,
														episode_flag=False,
														cumulative_flag=True,
														plot_title='Learning Curve',
														plot_flag=plot_flag)

				if j == len(data_name_array_auto)-1:

					current_path = data_path + data_name_auto

					if plot_path:
						observation_space_type = get_param_value_from_training_log(current_path, "observation_space_type")
						plt.plot(x_tot, y_tot, color_array[i], label=observation_space_type)

				get_training_result(data_path,
									data_name_auto,
									moving_average_window_size=plot_moving_average_window_size_timesteps,
									episode_flag=False,
									cumulative_flag=False,
									plot_title='Learning Curve',
									plot_flag=plot_flag)

				get_training_result(data_path,
									data_name_auto,
									moving_average_window_size=plot_moving_average_window_size_episodes,
									episode_flag=True,
									cumulative_flag=False,
									plot_title='Learning Curve',
									plot_flag=plot_flag)

				get_training_result(data_path,
									data_name_auto,
									moving_average_window_size=plot_moving_average_window_size_episodes,
									episode_flag=True,
									cumulative_flag=True,
									plot_title='Learning Curve',
									plot_flag=plot_flag)

		else:

			[x_tot, y_tot] = get_training_result(	data_path,
													data_name,
													moving_average_window_size=plot_moving_average_window_size_timesteps,
													cumulative_flag=True,
													plot_title='Learning Curve',
													plot_flag=plot_flag)
			
			if plot_path:
				observation_space_type = get_param_value_from_training_log(current_path, "observation_space_type")
				plt.plot(x_tot, y_tot, color_array[i], label=observation_space_type)

			get_training_result(data_path,
								data_name,
								moving_average_window_size=plot_moving_average_window_size_timesteps,
								episode_flag=False,
								cumulative_flag=False,
								plot_title='Learning Curve',
								plot_flag=plot_flag)

			get_training_result(data_path,
								data_name,
								moving_average_window_size=plot_moving_average_window_size_episodes,
								episode_flag=True,
								cumulative_flag=False,
								plot_title='Learning Curve',
								plot_flag=plot_flag)

			get_training_result(data_path,
								data_name,
								moving_average_window_size=plot_moving_average_window_size_episodes,
								episode_flag=True,
								cumulative_flag=True,
								plot_title='Learning Curve',
								plot_flag=plot_flag)
	
	if plot_path:
		plt.legend()
		plt.savefig(plot_path + "multi_cumulative_training_result.png")

	#plt.show()

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

	rospy.init_node('tentabot_drl_plot_result_training', anonymous=True, log_level=rospy.ERROR)

	data_cnt = rospy.get_param('data_cnt')

	data_path_array = []
	data_name_array = []
	for i in range(data_cnt):
		data_path_array.append(rospy.get_param('data_path' + str(i+1)))
		data_name_array.append(rospy.get_param('data_name' + str(i+1)))


	multi_flag = rospy.get_param('multi_flag')
	multi_plot_path = rospy.get_param('multi_plot_path')
	plot_flag = rospy.get_param('plot_flag')
	plot_path = rospy.get_param('plot_path')
	plot_title = rospy.get_param('plot_title')
	plot_moving_average_window_size_timesteps = rospy.get_param('plot_moving_average_window_size_timesteps')
	plot_moving_average_window_size_episodes = rospy.get_param('plot_moving_average_window_size_episodes')
	print_flag = rospy.get_param('print_flag')

	if multi_flag:

		plot_multi_training_result(	data_path_array=data_path_array, 
									data_name_array=data_name_array,
									plot_moving_average_window_size_timesteps=plot_moving_average_window_size_timesteps,
									plot_moving_average_window_size_episodes=plot_moving_average_window_size_episodes,
									plot_title='Learning Curve',
									plot_flag=plot_flag,
									plot_path=multi_plot_path)

	else:

		print_training_log(data_path_array[0] + data_name_array[0])

		# Plot timestep data
		get_training_result(data_path=data_path_array[0],
							data_name=data_name_array[0],
							moving_average_window_size=plot_moving_average_window_size_timesteps,
							episode_flag=False,
							cumulative_flag=False,
							print_flag=print_flag,
							plot_title=plot_title, 
							plot_flag=plot_flag, 
							plot_path=plot_path)
		
		get_training_result(data_path=data_path_array[0],
							data_name=data_name_array[0], 	
							moving_average_window_size=plot_moving_average_window_size_timesteps,
							episode_flag=False,
							cumulative_flag=True, 
							print_flag=print_flag,
							plot_title=plot_title,
							plot_flag=plot_flag, 
							plot_path=plot_path)

		# Plot episode data
		get_training_result(data_path=data_path_array[0],
							data_name=data_name_array[0],  
							moving_average_window_size=plot_moving_average_window_size_episodes,
							episode_flag=True,
							cumulative_flag=False,
							print_flag=print_flag,
							plot_title=plot_title,
							plot_flag=plot_flag, 
							plot_path=plot_path)

		get_training_result(data_path=data_path_array[0],
							data_name=data_name_array[0], 	
							moving_average_window_size=plot_moving_average_window_size_episodes,
							episode_flag=True,
							cumulative_flag=True,
							print_flag=print_flag, 
							plot_title=plot_title,
							plot_flag=plot_flag, 
							plot_path=plot_path)