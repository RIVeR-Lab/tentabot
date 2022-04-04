#!/usr/bin/env python3

'''
LAST UPDATE: 2021.10.10

AUTHOR: Neset Unver Akmandor (NUA)  
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
import xlwt
from xlwt import Workbook
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
def get_param_value_from_testing_log(log_path, param_name):

    with open(log_path + 'testing_input_log.csv') as csv_file:
        
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
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
def get_testing_result(log_path, plot_flag=False, plot_path=""):

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
    avg_duration = get_successful_avg(duration_data, success_data)
    avg_path_length = get_successful_avg(path_length_data, success_data)

    if plot_flag:
        plt.figure(1)
        plt.bar(label_x, success_data, color=color_array)
        plt.axhline(avg_success, color='blue', linewidth=2)
        plt.xlabel('Episode Index')
        plt.ylabel('Navigation Success')
        plt.title("Navigation Success")

        if plot_path:
            plt.savefig(plot_path + 'nav_success.png')
        else:
            plt.savefig(log_path + 'nav_success.png')
        
        plt.figure(2)
        plt.bar(label_x, duration_data, color=color_array)
        plt.axhline(avg_duration, color='blue', linewidth=2)
        plt.xlabel('Episode Index')
        plt.ylabel('Navigation Duration [s]')
        plt.title("Navigation Duration")

        if plot_path:
            plt.savefig(plot_path + 'nav_duration.png')
        else:
            plt.savefig(log_path + 'nav_duration.png')
        
        plt.figure(3)
        plt.bar(label_x, path_length_data, color=color_array)
        plt.axhline(avg_path_length, color='blue', linewidth=2)
        plt.xlabel('Episode Index')
        plt.ylabel('Navigation Path Length [m]')
        plt.title("Navigation Path Length")

        if plot_path:
            plt.savefig(plot_path + 'nav_path_length.png')
        else:
            plt.savefig(log_path + 'nav_path_length.png')

    return [avg_success, avg_duration, avg_path_length]

'''
DESCRIPTION: TODO...
'''
def get_multi_testing_result(dataset_path_array, multi_result_path=""):

    wb = Workbook()
    testing_result = wb.add_sheet('Testing Result')
    testing_result_overall = wb.add_sheet('Overall Testing Result')
    testing_result_overall.write(1, 0, 'Success')
    testing_result_overall.write(2, 0, 'Duration')
    testing_result_overall.write(3, 0, 'Path Length')
    row_dict = {}

    for i, dataset_path in enumerate(dataset_path_array):

        r = 1
        avg_multi_success = 0
        avg_multi_duration = 0
        avg_multi_path_length = 0
        
        dataset = [x[0] for x in os.walk(dataset_path)]

        for j, data_path in enumerate(dataset):

            if j == 0:
                dataset_name = data_path.split("/")[-2]
                testing_result.write(0, i+2, str(dataset_name))
                testing_result_overall.write(0, i+1, str(dataset_name))

            else:
                [success, duration, path_length] = get_testing_result(data_path + '/')
                world_name = get_param_value_from_testing_log(data_path + '/', 'world_name')

                if i == 0:
                    row_dict[world_name] = r
                    testing_result.write(r, 0, world_name)
                    testing_result.write(r, 1, 'Success')
                    testing_result.write(r+1, 1, 'Duration')
                    testing_result.write(r+2, 1, 'Path Length')

                else:
                    r = row_dict[world_name]

                testing_result.write(r, i+2, str(round(success, 2)))
                testing_result.write(r+1, i+2, str(round(duration, 2)))
                testing_result.write(r+2, i+2, str(round(path_length, 2)))

                avg_multi_success += success
                avg_multi_duration += duration
                avg_multi_path_length += path_length

                r += 4

                #print(str(j) + " -> " + data_path)
        #print("")

        avg_multi_success /= (len(dataset)-1)
        avg_multi_duration /= (len(dataset)-1)
        avg_multi_path_length /= (len(dataset)-1)

        testing_result_overall.write(1, i+1, str(round(avg_multi_success, 2)))
        testing_result_overall.write(2, i+1, str(round(avg_multi_duration, 2)))
        testing_result_overall.write(3, i+1, str(round(avg_multi_path_length, 2)))

    wb.save(multi_result_path + 'testing_result.xls')

'''
DESCRIPTION: TODO...
'''
if __name__ == '__main__':

    rospy.init_node('plot_drl_testing_result', anonymous=True, log_level=rospy.ERROR)

    plot_flag = rospy.get_param('plot_flag')
    plot_path = rospy.get_param('plot_path')
    print_data = rospy.get_param('print_data')
    multi_data_flag = rospy.get_param('multi_data_flag')

    if not multi_data_flag:

        data_path = rospy.get_param('data_path')

        if print_data:
            print_testing_result_log(data_path)

        get_testing_result(data_path, plot_flag=plot_flag, plot_path=plot_path)
    
    else:

        multi_result_path = rospy.get_param('multi_result_path')
        multi_data_cnt = rospy.get_param('multi_data_cnt')
        
        dataset_path_array = []
        for i in range(multi_data_cnt):
            dataset_path_array.append(rospy.get_param('multi_data_path' + str(i+1)))
        
        get_multi_testing_result(dataset_path_array, multi_result_path=multi_result_path)