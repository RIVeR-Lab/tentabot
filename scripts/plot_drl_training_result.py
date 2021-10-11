#!/usr/bin/env python3

'''
LAST UPDATE: 2021.10.10

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
		print("tentabot_rl_plot_training_results::print_training_log ->")
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

	:param log_path: (str) the save location of the results to plot
	:param title: (str) the title of the task to plot
'''
def get_training_result(log_path, title='Learning Curve', moving_average_window_size=5, print_data=False, plot_flag=False, plot_path=""):

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

	if plot_flag:
		plt.figure()
		plt.plot(x, y)
		plt.xlabel('Number of Timesteps')
		plt.ylabel('Rewards')
		plt.title(title + " (smoothed)")

		if plot_path:
			plt.savefig(plot_path + "training_result.png")
		else:
			plt.savefig(log_path + "training_result.png")

	return [x, y]


'''
DESCRIPTION: TODO...plot the results

	:param log_path: (str) the save location of the results to plot
	:param title: (str) the title of the task to plot
'''
def get_cumulative_training_result(log_path, title='Learning Curve', label="", moving_average_window_size=5, print_data=False, plot_flag=False, plot_path=""):

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

	if plot_flag:
		plt.figure()
		plt.plot(x_tot, y_tot, label=label)
		plt.xlabel('Number of Timesteps')
		plt.ylabel('Rewards')
		plt.title("Cumulative " + title + " (smoothed)")

		if plot_path:
			plt.savefig(plot_path + "cumulative_training_result.png")
		else:
			plt.savefig(log_path + "cumulative_training_result.png")

	return [x_tot, y_tot]

'''
DESCRIPTION: TODO...
'''
def plot_multi_cumulative_training_result(log_path_array, title='Learning Curve', moving_average_window_size=5, plot_path=""):

	color_array = ['r', 'g', 'b', 'y', 'm', 'c']

	#plt.figure()
	plt.xlabel('Number of Timesteps')
	plt.ylabel('Rewards')
	plt.title("Multi Cumulative " + title + " (smoothed)")

	for i, log_path in enumerate(log_path_array):

		[x_tot, y_tot] = get_cumulative_training_result(log_path, title='Learning Curve', moving_average_window_size=moving_average_window_size)
		observation_space_type = get_param_value_from_training_log(log_path, "observation_space_type")
		plt.plot(x_tot, y_tot, color_array[i], label=observation_space_type)
		
	plt.legend()
	plt.savefig(plot_path + "multi_cumulative_training_result.png")

	#plt.show()

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

	rospy.init_node('plot_drl_training_result', anonymous=True, log_level=rospy.ERROR)

	data_cnt = rospy.get_param('data_cnt')

	data_path_array = []
	for i in range(data_cnt):
		data_path_array.append(rospy.get_param('data_path' + str(i+1)))

	plot_flag = rospy.get_param('plot_flag')
	plot_path = rospy.get_param('plot_path')
	plot_title = rospy.get_param('plot_title')
	plot_moving_average_window_size = rospy.get_param('plot_moving_average_window_size')
	print_data = rospy.get_param('print_data')

	if data_cnt == 1:
		print_training_log(data_path_array[0])
		get_training_result(data_path_array[0], title=plot_title, moving_average_window_size=plot_moving_average_window_size, print_data=print_data, plot_flag=plot_flag, plot_path=plot_path)
		get_cumulative_training_result(data_path_array[0], title=plot_title, moving_average_window_size=plot_moving_average_window_size, print_data=print_data, plot_flag=plot_flag, plot_path=plot_path)

	elif data_cnt > 1:
		plot_multi_cumulative_training_result(data_path_array, title='Learning Curve', moving_average_window_size=plot_moving_average_window_size, plot_path=plot_path)