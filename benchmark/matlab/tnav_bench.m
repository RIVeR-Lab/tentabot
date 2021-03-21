close all
clear
clc

path_bench = "/home/akmandor/catkin_ws/src/tentabot/benchmark/tnav/";

path_c0_ewok_C7 = path_bench + "ewok_cylinder0_C7/";
path_f0_ewok_C7 = path_bench + "ewok_forest0_C7/";
path_f1_ewok_C7 = path_bench + "ewok_forest1_C7/";
path_f2_ewok_C7 = path_bench + "ewok_forest2_C7/";
path_f3_ewok_C7 = path_bench + "ewok_forest3_C7/";
path_f4_ewok_C7 = path_bench + "ewok_forest4_C7/";
path_f5_ewok_C7 = path_bench + "ewok_forest5_C7/";
path_f7_ewok_C7 = path_bench + "ewok_forest7_C7/";
path_f8_ewok_C7 = path_bench + "ewok_forest8_C7/";
path_f9_ewok_C7 = path_bench + "ewok_forest9_C7/";

path_c0_ewok_C9 = path_bench + "ewok_cylinder0_C9/";
path_f0_ewok_C9 = path_bench + "ewok_forest0_C9/";
path_f1_ewok_C9 = path_bench + "ewok_forest1_C9/";
path_f2_ewok_C9 = path_bench + "ewok_forest2_C9/";
path_f3_ewok_C9 = path_bench + "ewok_forest3_C9/";
path_f4_ewok_C9 = path_bench + "ewok_forest4_C9/";
path_f5_ewok_C9 = path_bench + "ewok_forest5_C9/";
path_f7_ewok_C9 = path_bench + "ewok_forest7_C9/";
path_f8_ewok_C9 = path_bench + "ewok_forest8_C9/";
path_f9_ewok_C9 = path_bench + "ewok_forest9_C9/";

path_c0_set1 = path_bench + "tentabot_2019_set1_cylinder0/";
path_f0_set1 = path_bench + "tentabot_2019_set1_forest0/";
path_f1_set1 = path_bench + "tentabot_2019_set1_forest1/";
path_f2_set1 = path_bench + "tentabot_2019_set1_forest2/";
path_f3_set1 = path_bench + "tentabot_2019_set1_forest3/";
path_f4_set1 = path_bench + "tentabot_2019_set1_forest4/";
path_f5_set1 = path_bench + "tentabot_2019_set1_forest5/";
path_f7_set1 = path_bench + "tentabot_2019_set1_forest7/";
path_f8_set1 = path_bench + "tentabot_2019_set1_forest8/";
path_f9_set1 = path_bench + "tentabot_2019_set1_forest9/";

path_c0_set1_tbot2021 = path_bench + "tentabot_2021_set1_cylinder0/";
path_f0_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest0/";
path_f1_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest1/";
path_f2_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest2/";
path_f3_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest3/";
path_f4_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest4/";
path_f5_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest5/";
path_f7_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest7/";
path_f8_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest8/";
path_f9_set1_tbot2021 = path_bench + "tentabot_2021_set1_forest9/";

path_c0_set2_tbot2021 = path_bench + "tentabot_2021_set2_cylinder0/";

path_c0_set3_tbot2021 = path_bench + "tentabot_2021_set3_cylinder0/";

%% EXTRACT EWOK RESULT BENCHMARKS
[success_rate_ewokC7_c0, mean_duration_ewokC7_c0, var_duration_ewokC7_c0, mean_length_ewokC7_c0, var_length_ewokC7_c0] = tnav_bench_extract_result(path_c0_ewok_C7);
[success_rate_ewokC7_f0, mean_duration_ewokC7_f0, var_duration_ewokC7_f0, mean_length_ewokC7_f0, var_length_ewokC7_f0] = tnav_bench_extract_result(path_f0_ewok_C7);
[success_rate_ewokC7_f1, mean_duration_ewokC7_f1, var_duration_ewokC7_f1, mean_length_ewokC7_f1, var_length_ewokC7_f1] = tnav_bench_extract_result(path_f1_ewok_C7);
[success_rate_ewokC7_f2, mean_duration_ewokC7_f2, var_duration_ewokC7_f2, mean_length_ewokC7_f2, var_length_ewokC7_f2] = tnav_bench_extract_result(path_f2_ewok_C7);
[success_rate_ewokC7_f3, mean_duration_ewokC7_f3, var_duration_ewokC7_f3, mean_length_ewokC7_f3, var_length_ewokC7_f3] = tnav_bench_extract_result(path_f3_ewok_C7);
[success_rate_ewokC7_f4, mean_duration_ewokC7_f4, var_duration_ewokC7_f4, mean_length_ewokC7_f4, var_length_ewokC7_f4] = tnav_bench_extract_result(path_f4_ewok_C7);
[success_rate_ewokC7_f5, mean_duration_ewokC7_f5, var_duration_ewokC7_f5, mean_length_ewokC7_f5, var_length_ewokC7_f5] = tnav_bench_extract_result(path_f5_ewok_C7);
[success_rate_ewokC7_f7, mean_duration_ewokC7_f7, var_duration_ewokC7_f7, mean_length_ewokC7_f7, var_length_ewokC7_f7] = tnav_bench_extract_result(path_f7_ewok_C7);
[success_rate_ewokC7_f8, mean_duration_ewokC7_f8, var_duration_ewokC7_f8, mean_length_ewokC7_f8, var_length_ewokC7_f8] = tnav_bench_extract_result(path_f8_ewok_C7);
[success_rate_ewokC7_f9, mean_duration_ewokC7_f9, var_duration_ewokC7_f9, mean_length_ewokC7_f9, var_length_ewokC7_f9] = tnav_bench_extract_result(path_f9_ewok_C7);

[success_rate_ewokC9_c0, mean_duration_ewokC9_c0, var_duration_ewokC9_c0, mean_length_ewokC9_c0, var_length_ewokC9_c0] = tnav_bench_extract_result(path_c0_ewok_C9);
[success_rate_ewokC9_f0, mean_duration_ewokC9_f0, var_duration_ewokC9_f0, mean_length_ewokC9_f0, var_length_ewokC9_f0] = tnav_bench_extract_result(path_f0_ewok_C9);
[success_rate_ewokC9_f1, mean_duration_ewokC9_f1, var_duration_ewokC9_f1, mean_length_ewokC9_f1, var_length_ewokC9_f1] = tnav_bench_extract_result(path_f1_ewok_C9);
[success_rate_ewokC9_f2, mean_duration_ewokC9_f2, var_duration_ewokC9_f2, mean_length_ewokC9_f2, var_length_ewokC9_f2] = tnav_bench_extract_result(path_f2_ewok_C9);
[success_rate_ewokC9_f3, mean_duration_ewokC9_f3, var_duration_ewokC9_f3, mean_length_ewokC9_f3, var_length_ewokC9_f3] = tnav_bench_extract_result(path_f3_ewok_C9);
[success_rate_ewokC9_f4, mean_duration_ewokC9_f4, var_duration_ewokC9_f4, mean_length_ewokC9_f4, var_length_ewokC9_f4] = tnav_bench_extract_result(path_f4_ewok_C9);
[success_rate_ewokC9_f5, mean_duration_ewokC9_f5, var_duration_ewokC9_f5, mean_length_ewokC9_f5, var_length_ewokC9_f5] = tnav_bench_extract_result(path_f5_ewok_C9);
[success_rate_ewokC9_f7, mean_duration_ewokC9_f7, var_duration_ewokC9_f7, mean_length_ewokC9_f7, var_length_ewokC9_f7] = tnav_bench_extract_result(path_f7_ewok_C9);
[success_rate_ewokC9_f8, mean_duration_ewokC9_f8, var_duration_ewokC9_f8, mean_length_ewokC9_f8, var_length_ewokC9_f8] = tnav_bench_extract_result(path_f8_ewok_C9);
[success_rate_ewokC9_f9, mean_duration_ewokC9_f9, var_duration_ewokC9_f9, mean_length_ewokC9_f9, var_length_ewokC9_f9] = tnav_bench_extract_result(path_f9_ewok_C9);

%% EXTRACT TENTABOT PRE BENCHMARKS
[mean_grid_c0_tbot, var_grid_c0_tbot, mean_tentacle_voxel_c0_tbot, var_tentacle_voxel_c0_tbot] = tnav_bench_extract_pre(path_c0_set1);
[mean_grid_f0_tbot, var_grid_f0_tbot, mean_tentacle_voxel_f0_tbot, var_tentacle_voxel_f0_tbot] = tnav_bench_extract_pre(path_f0_set1);
[mean_grid_f1_tbot, var_grid_f1_tbot, mean_tentacle_voxel_f1_tbot, var_tentacle_voxel_f1_tbot] = tnav_bench_extract_pre(path_f1_set1);
[mean_grid_f2_tbot, var_grid_f2_tbot, mean_tentacle_voxel_f2_tbot, var_tentacle_voxel_f2_tbot] = tnav_bench_extract_pre(path_f2_set1);
[mean_grid_f3_tbot, var_grid_f3_tbot, mean_tentacle_voxel_f3_tbot, var_tentacle_voxel_f3_tbot] = tnav_bench_extract_pre(path_f3_set1);
[mean_grid_f4_tbot, var_grid_f4_tbot, mean_tentacle_voxel_f4_tbot, var_tentacle_voxel_f4_tbot] = tnav_bench_extract_pre(path_f4_set1);
[mean_grid_f5_tbot, var_grid_f5_tbot, mean_tentacle_voxel_f5_tbot, var_tentacle_voxel_f5_tbot] = tnav_bench_extract_pre(path_f5_set1);
[mean_grid_f7_tbot, var_grid_f7_tbot, mean_tentacle_voxel_f7_tbot, var_tentacle_voxel_f7_tbot] = tnav_bench_extract_pre(path_f7_set1);
[mean_grid_f8_tbot, var_grid_f8_tbot, mean_tentacle_voxel_f8_tbot, var_tentacle_voxel_f8_tbot] = tnav_bench_extract_pre(path_f8_set1);
[mean_grid_f9_tbot, var_grid_f9_tbot, mean_tentacle_voxel_f9_tbot, var_tentacle_voxel_f9_tbot] = tnav_bench_extract_pre(path_f9_set1);

[mean_grid_c0_tbot2021, var_grid_c0_tbot2021, mean_tentacle_voxel_c0_tbot2021, var_tentacle_voxel_c0_tbot2021] = tnav_bench_extract_pre(path_c0_set1_tbot2021);
[mean_grid_f0_tbot2021, var_grid_f0_tbot2021, mean_tentacle_voxel_f0_tbot2021, var_tentacle_voxel_f0_tbot2021] = tnav_bench_extract_pre(path_f0_set1_tbot2021);
[mean_grid_f1_tbot2021, var_grid_f1_tbot2021, mean_tentacle_voxel_f1_tbot2021, var_tentacle_voxel_f1_tbot2021] = tnav_bench_extract_pre(path_f1_set1_tbot2021);
[mean_grid_f2_tbot2021, var_grid_f2_tbot2021, mean_tentacle_voxel_f2_tbot2021, var_tentacle_voxel_f2_tbot2021] = tnav_bench_extract_pre(path_f2_set1_tbot2021);
[mean_grid_f3_tbot2021, var_grid_f3_tbot2021, mean_tentacle_voxel_f3_tbot2021, var_tentacle_voxel_f3_tbot2021] = tnav_bench_extract_pre(path_f3_set1_tbot2021);
[mean_grid_f4_tbot2021, var_grid_f4_tbot2021, mean_tentacle_voxel_f4_tbot2021, var_tentacle_voxel_f4_tbot2021] = tnav_bench_extract_pre(path_f4_set1_tbot2021);
[mean_grid_f5_tbot2021, var_grid_f5_tbot2021, mean_tentacle_voxel_f5_tbot2021, var_tentacle_voxel_f5_tbot2021] = tnav_bench_extract_pre(path_f5_set1_tbot2021);
[mean_grid_f7_tbot2021, var_grid_f7_tbot2021, mean_tentacle_voxel_f7_tbot2021, var_tentacle_voxel_f7_tbot2021] = tnav_bench_extract_pre(path_f7_set1_tbot2021);
[mean_grid_f8_tbot2021, var_grid_f8_tbot2021, mean_tentacle_voxel_f8_tbot2021, var_tentacle_voxel_f8_tbot2021] = tnav_bench_extract_pre(path_f8_set1_tbot2021);
[mean_grid_f9_tbot2021, var_grid_f9_tbot2021, mean_tentacle_voxel_f9_tbot2021, var_tentacle_voxel_f9_tbot2021] = tnav_bench_extract_pre(path_f9_set1_tbot2021);

[mean_grid_c0_set2_tbot, var_grid_c0_set2_tbot, mean_tentacle_voxel_c0_set2_tbot, var_tentacle_voxel_c0_set2_tbot] = tnav_bench_extract_pre(path_c0_set2_tbot2021);

[mean_grid_c0_set3_tbot, var_grid_c0_set3_tbot, mean_tentacle_voxel_c0_set3_tbot, var_tentacle_voxel_c0_set3_tbot] = tnav_bench_extract_pre(path_c0_set3_tbot2021);

%% EXTRACT TENTABOT PROCESS BENCHMARKS

[avg_mean_upGVox_c0_tbot, avg_var_upGVox_c0_tbot, avg_mean_upHeur_c0_tbot, avg_var_upHeur_c0_tbot, avg_mean_selectT_c0_tbot, avg_var_selectT_c0_tbot, avg_mean_moveT_c0_tbot, avg_var_moveT_c0_tbot] = tnav_bench_extract_process(path_c0_set1);
[avg_mean_upGVox_f0_tbot, avg_var_upGVox_f0_tbot, avg_mean_upHeur_f0_tbot, avg_var_upHeur_f0_tbot, avg_mean_selectT_f0_tbot, avg_var_selectT_f0_tbot, avg_mean_moveT_f0_tbot, avg_var_moveT_f0_tbot] = tnav_bench_extract_process(path_f0_set1);
[avg_mean_upGVox_f1_tbot, avg_var_upGVox_f1_tbot, avg_mean_upHeur_f1_tbot, avg_var_upHeur_f1_tbot, avg_mean_selectT_f1_tbot, avg_var_selectT_f1_tbot, avg_mean_moveT_f1_tbot, avg_var_moveT_f1_tbot] = tnav_bench_extract_process(path_f1_set1);
[avg_mean_upGVox_f2_tbot, avg_var_upGVox_f2_tbot, avg_mean_upHeur_f2_tbot, avg_var_upHeur_f2_tbot, avg_mean_selectT_f2_tbot, avg_var_selectT_f2_tbot, avg_mean_moveT_f2_tbot, avg_var_moveT_f2_tbot] = tnav_bench_extract_process(path_f2_set1);
[avg_mean_upGVox_f3_tbot, avg_var_upGVox_f3_tbot, avg_mean_upHeur_f3_tbot, avg_var_upHeur_f3_tbot, avg_mean_selectT_f3_tbot, avg_var_selectT_f3_tbot, avg_mean_moveT_f3_tbot, avg_var_moveT_f3_tbot] = tnav_bench_extract_process(path_f3_set1);
[avg_mean_upGVox_f4_tbot, avg_var_upGVox_f4_tbot, avg_mean_upHeur_f4_tbot, avg_var_upHeur_f4_tbot, avg_mean_selectT_f4_tbot, avg_var_selectT_f4_tbot, avg_mean_moveT_f4_tbot, avg_var_moveT_f4_tbot] = tnav_bench_extract_process(path_f4_set1);
[avg_mean_upGVox_f5_tbot, avg_var_upGVox_f5_tbot, avg_mean_upHeur_f5_tbot, avg_var_upHeur_f5_tbot, avg_mean_selectT_f5_tbot, avg_var_selectT_f5_tbot, avg_mean_moveT_f5_tbot, avg_var_moveT_f5_tbot] = tnav_bench_extract_process(path_f5_set1);
[avg_mean_upGVox_f7_tbot, avg_var_upGVox_f7_tbot, avg_mean_upHeur_f7_tbot, avg_var_upHeur_f7_tbot, avg_mean_selectT_f7_tbot, avg_var_selectT_f7_tbot, avg_mean_moveT_f7_tbot, avg_var_moveT_f7_tbot] = tnav_bench_extract_process(path_f7_set1);
[avg_mean_upGVox_f8_tbot, avg_var_upGVox_f8_tbot, avg_mean_upHeur_f8_tbot, avg_var_upHeur_f8_tbot, avg_mean_selectT_f8_tbot, avg_var_selectT_f8_tbot, avg_mean_moveT_f8_tbot, avg_var_moveT_f8_tbot] = tnav_bench_extract_process(path_f8_set1);
[avg_mean_upGVox_f9_tbot, avg_var_upGVox_f9_tbot, avg_mean_upHeur_f9_tbot, avg_var_upHeur_f9_tbot, avg_mean_selectT_f9_tbot, avg_var_selectT_f9_tbot, avg_mean_moveT_f9_tbot, avg_var_moveT_f9_tbot] = tnav_bench_extract_process(path_f9_set1);

[avg_mean_upGVox_c0_tbot2021, avg_var_upGVox_c0_tbot2021, avg_mean_upHeur_c0_tbot2021, avg_var_upHeur_c0_tbot2021, avg_mean_selectT_c0_tbot2021, avg_var_selectT_c0_tbot2021, avg_mean_moveT_c0_tbot2021, avg_var_moveT_c0_tbot2021] = tnav_bench_extract_process(path_c0_set1_tbot2021);
[avg_mean_upGVox_f0_tbot2021, avg_var_upGVox_f0_tbot2021, avg_mean_upHeur_f0_tbot2021, avg_var_upHeur_f0_tbot2021, avg_mean_selectT_f0_tbot2021, avg_var_selectT_f0_tbot2021, avg_mean_moveT_f0_tbot2021, avg_var_moveT_f0_tbot2021] = tnav_bench_extract_process(path_f0_set1_tbot2021);
[avg_mean_upGVox_f1_tbot2021, avg_var_upGVox_f1_tbot2021, avg_mean_upHeur_f1_tbot2021, avg_var_upHeur_f1_tbot2021, avg_mean_selectT_f1_tbot2021, avg_var_selectT_f1_tbot2021, avg_mean_moveT_f1_tbot2021, avg_var_moveT_f1_tbot2021] = tnav_bench_extract_process(path_f1_set1_tbot2021);
[avg_mean_upGVox_f2_tbot2021, avg_var_upGVox_f2_tbot2021, avg_mean_upHeur_f2_tbot2021, avg_var_upHeur_f2_tbot2021, avg_mean_selectT_f2_tbot2021, avg_var_selectT_f2_tbot2021, avg_mean_moveT_f2_tbot2021, avg_var_moveT_f2_tbot2021] = tnav_bench_extract_process(path_f2_set1_tbot2021);
[avg_mean_upGVox_f3_tbot2021, avg_var_upGVox_f3_tbot2021, avg_mean_upHeur_f3_tbot2021, avg_var_upHeur_f3_tbot2021, avg_mean_selectT_f3_tbot2021, avg_var_selectT_f3_tbot2021, avg_mean_moveT_f3_tbot2021, avg_var_moveT_f3_tbot2021] = tnav_bench_extract_process(path_f3_set1_tbot2021);
[avg_mean_upGVox_f4_tbot2021, avg_var_upGVox_f4_tbot2021, avg_mean_upHeur_f4_tbot2021, avg_var_upHeur_f4_tbot2021, avg_mean_selectT_f4_tbot2021, avg_var_selectT_f4_tbot2021, avg_mean_moveT_f4_tbot2021, avg_var_moveT_f4_tbot2021] = tnav_bench_extract_process(path_f4_set1_tbot2021);
[avg_mean_upGVox_f5_tbot2021, avg_var_upGVox_f5_tbot2021, avg_mean_upHeur_f5_tbot2021, avg_var_upHeur_f5_tbot2021, avg_mean_selectT_f5_tbot2021, avg_var_selectT_f5_tbot2021, avg_mean_moveT_f5_tbot2021, avg_var_moveT_f5_tbot2021] = tnav_bench_extract_process(path_f5_set1_tbot2021);
[avg_mean_upGVox_f7_tbot2021, avg_var_upGVox_f7_tbot2021, avg_mean_upHeur_f7_tbot2021, avg_var_upHeur_f7_tbot2021, avg_mean_selectT_f7_tbot2021, avg_var_selectT_f7_tbot2021, avg_mean_moveT_f7_tbot2021, avg_var_moveT_f7_tbot2021] = tnav_bench_extract_process(path_f7_set1_tbot2021);
[avg_mean_upGVox_f8_tbot2021, avg_var_upGVox_f8_tbot2021, avg_mean_upHeur_f8_tbot2021, avg_var_upHeur_f8_tbot2021, avg_mean_selectT_f8_tbot2021, avg_var_selectT_f8_tbot2021, avg_mean_moveT_f8_tbot2021, avg_var_moveT_f8_tbot2021] = tnav_bench_extract_process(path_f8_set1_tbot2021);
[avg_mean_upGVox_f9_tbot2021, avg_var_upGVox_f9_tbot2021, avg_mean_upHeur_f9_tbot2021, avg_var_upHeur_f9_tbot2021, avg_mean_selectT_f9_tbot2021, avg_var_selectT_f9_tbot2021, avg_mean_moveT_f9_tbot2021, avg_var_moveT_f9_tbot2021] = tnav_bench_extract_process(path_f9_set1_tbot2021);

[avg_mean_upGVox_c0_set2_tbot, avg_var_upGVox_c0_set2_tbot, avg_mean_upHeur_c0_set2_tbot, avg_var_upHeur_c0_set2_tbot, avg_mean_selectT_c0_set2_tbot, avg_var_selectT_c0_set2_tbot, avg_mean_moveT_c0_set2_tbot, avg_var_moveT_c0_set2_tbot] = tnav_bench_extract_process(path_c0_set2_tbot2021);

[avg_mean_upGVox_c0_set3_tbot, avg_var_upGVox_c0_set3_tbot, avg_mean_upHeur_c0_set3_tbot, avg_var_upHeur_c0_set3_tbot, avg_mean_selectT_c0_set3_tbot, avg_var_selectT_c0_set3_tbot, avg_mean_moveT_c0_set3_tbot, avg_var_moveT_c0_set3_tbot] = tnav_bench_extract_process(path_c0_set3_tbot2021);

%% EXTRACT TENTABOT RESULT BENCHMARKS
[success_rate_tbot_c0, mean_duration_tbot_c0, var_duration_tbot_c0, mean_length_tbot_c0, var_length_tbot_c0] = tnav_bench_extract_result(path_c0_set1);
[success_rate_tbot_f0, mean_duration_tbot_f0, var_duration_tbot_f0, mean_length_tbot_f0, var_length_tbot_f0] = tnav_bench_extract_result(path_f0_set1);
[success_rate_tbot_f1, mean_duration_tbot_f1, var_duration_tbot_f1, mean_length_tbot_f1, var_length_tbot_f1] = tnav_bench_extract_result(path_f1_set1);
[success_rate_tbot_f2, mean_duration_tbot_f2, var_duration_tbot_f2, mean_length_tbot_f2, var_length_tbot_f2] = tnav_bench_extract_result(path_f2_set1);
[success_rate_tbot_f3, mean_duration_tbot_f3, var_duration_tbot_f3, mean_length_tbot_f3, var_length_tbot_f3] = tnav_bench_extract_result(path_f3_set1);
[success_rate_tbot_f4, mean_duration_tbot_f4, var_duration_tbot_f4, mean_length_tbot_f4, var_length_tbot_f4] = tnav_bench_extract_result(path_f4_set1);
[success_rate_tbot_f5, mean_duration_tbot_f5, var_duration_tbot_f5, mean_length_tbot_f5, var_length_tbot_f5] = tnav_bench_extract_result(path_f5_set1);
[success_rate_tbot_f7, mean_duration_tbot_f7, var_duration_tbot_f7, mean_length_tbot_f7, var_length_tbot_f7] = tnav_bench_extract_result(path_f7_set1);
[success_rate_tbot_f8, mean_duration_tbot_f8, var_duration_tbot_f8, mean_length_tbot_f8, var_length_tbot_f8] = tnav_bench_extract_result(path_f8_set1);
[success_rate_tbot_f9, mean_duration_tbot_f9, var_duration_tbot_f9, mean_length_tbot_f9, var_length_tbot_f9] = tnav_bench_extract_result(path_f9_set1);

[success_rate_tbot2021_c0, mean_duration_tbot2021_c0, var_duration_tbot2021_c0, mean_length_tbot2021_c0, var_length_tbot2021_c0] = tnav_bench_extract_result(path_c0_set1_tbot2021);
[success_rate_tbot2021_f0, mean_duration_tbot2021_f0, var_duration_tbot2021_f0, mean_length_tbot2021_f0, var_length_tbot2021_f0] = tnav_bench_extract_result(path_f0_set1_tbot2021);
[success_rate_tbot2021_f1, mean_duration_tbot2021_f1, var_duration_tbot2021_f1, mean_length_tbot2021_f1, var_length_tbot2021_f1] = tnav_bench_extract_result(path_f1_set1_tbot2021);
[success_rate_tbot2021_f2, mean_duration_tbot2021_f2, var_duration_tbot2021_f2, mean_length_tbot2021_f2, var_length_tbot2021_f2] = tnav_bench_extract_result(path_f2_set1_tbot2021);
[success_rate_tbot2021_f3, mean_duration_tbot2021_f3, var_duration_tbot2021_f3, mean_length_tbot2021_f3, var_length_tbot2021_f3] = tnav_bench_extract_result(path_f3_set1_tbot2021);
[success_rate_tbot2021_f4, mean_duration_tbot2021_f4, var_duration_tbot2021_f4, mean_length_tbot2021_f4, var_length_tbot2021_f4] = tnav_bench_extract_result(path_f4_set1_tbot2021);
[success_rate_tbot2021_f5, mean_duration_tbot2021_f5, var_duration_tbot2021_f5, mean_length_tbot2021_f5, var_length_tbot2021_f5] = tnav_bench_extract_result(path_f5_set1_tbot2021);
[success_rate_tbot2021_f7, mean_duration_tbot2021_f7, var_duration_tbot2021_f7, mean_length_tbot2021_f7, var_length_tbot2021_f7] = tnav_bench_extract_result(path_f7_set1_tbot2021);
[success_rate_tbot2021_f8, mean_duration_tbot2021_f8, var_duration_tbot2021_f8, mean_length_tbot2021_f8, var_length_tbot2021_f8] = tnav_bench_extract_result(path_f8_set1_tbot2021);
[success_rate_tbot2021_f9, mean_duration_tbot2021_f9, var_duration_tbot2021_f9, mean_length_tbot2021_f9, var_length_tbot2021_f9] = tnav_bench_extract_result(path_f9_set1_tbot2021);

%% DISPLAY PRE-NAVIGATION AVERAGE COMPUTATION TIMES
mean_grid_all_tbot = [mean_grid_c0_tbot2021, mean_grid_f0_tbot2021, mean_grid_f1_tbot2021, mean_grid_f2_tbot2021, mean_grid_f3_tbot2021, mean_grid_f4_tbot2021, mean_grid_f5_tbot2021, mean_grid_f7_tbot2021, mean_grid_f8_tbot2021, mean_grid_f9_tbot2021];
var_grid_all_tbot = [var_grid_c0_tbot2021, var_grid_f0_tbot2021, var_grid_f1_tbot2021, var_grid_f2_tbot2021, var_grid_f3_tbot2021, var_grid_f4_tbot2021, var_grid_f5_tbot2021, var_grid_f7_tbot2021, var_grid_f8_tbot2021, var_grid_f9_tbot2021];

mean_tentacle_voxel_all_tbot = [mean_tentacle_voxel_c0_tbot2021, mean_tentacle_voxel_f0_tbot2021, mean_tentacle_voxel_f1_tbot2021, mean_tentacle_voxel_f2_tbot2021, mean_tentacle_voxel_f3_tbot2021, mean_tentacle_voxel_f4_tbot2021, mean_tentacle_voxel_f5_tbot2021, mean_tentacle_voxel_f7_tbot2021, mean_tentacle_voxel_f8_tbot2021, mean_tentacle_voxel_f9_tbot2021];
var_tentacle_voxel_all_tbot = [var_tentacle_voxel_c0_tbot2021, var_tentacle_voxel_f0_tbot2021, var_tentacle_voxel_f1_tbot2021, var_tentacle_voxel_f2_tbot2021, var_tentacle_voxel_f3_tbot2021, var_tentacle_voxel_f4_tbot2021, var_tentacle_voxel_f5_tbot2021, var_tentacle_voxel_f7_tbot2021, var_tentacle_voxel_f8_tbot2021, var_tentacle_voxel_f9_tbot2021];

avg_mean_grid_all_tbot = mean(mean_grid_all_tbot);
avg_mean_tentacle_voxel_all_tbot = mean(mean_tentacle_voxel_all_tbot);
total_pre = avg_mean_grid_all_tbot + avg_mean_tentacle_voxel_all_tbot;

disp("Pre-navigation Average Computation Times: set1");
disp("Forming the robot-centered grid structure [s]: Mean = " + avg_mean_grid_all_tbot + ", Var = " + mean(var_grid_all_tbot));
disp("Forming the tentacles and extracting voxels [s]: Mean = " + avg_mean_tentacle_voxel_all_tbot + ", Var = " + mean(var_tentacle_voxel_all_tbot));
disp("Total (mean) [s] = " + total_pre);
disp(" ");

total_pre = mean_grid_c0_set2_tbot + mean_tentacle_voxel_c0_set2_tbot;

disp("Pre-navigation Average Computation Times: set2");
disp("Forming the robot-centered grid structure [s]: Mean = " + mean_grid_c0_set2_tbot + ", Var = " + var_grid_c0_set2_tbot);
disp("Forming the tentacles and extracting voxels [s]: Mean = " + mean_tentacle_voxel_c0_set2_tbot + ", Var = " + var_tentacle_voxel_c0_set2_tbot);
disp("Total (mean) [s] = " + total_pre);
disp(" ");

total_pre = mean_grid_c0_set3_tbot + mean_tentacle_voxel_c0_set3_tbot;

disp("Pre-navigation Average Computation Times: set3");
disp("Forming the robot-centered grid structure [s]: Mean = " + mean_grid_c0_set3_tbot + ", Var = " + var_grid_c0_set3_tbot);
disp("Forming the tentacles and extracting voxels [s]: Mean = " + mean_tentacle_voxel_c0_set3_tbot + ", Var = " + var_tentacle_voxel_c0_set3_tbot);
disp("Total (mean) [s] = " + total_pre);
disp(" ");

%% DISPLAY DURING-NAVIGATION AVERAGE COMPUTATION TIMES
avg_mean_upGVox_all_tbot = 10^(-6) * mean([avg_mean_upGVox_c0_tbot2021, avg_mean_upGVox_f0_tbot2021, avg_mean_upGVox_f1_tbot2021, avg_mean_upGVox_f2_tbot2021, avg_mean_upGVox_f3_tbot2021, avg_mean_upGVox_f4_tbot2021, avg_mean_upGVox_f5_tbot2021, avg_mean_upGVox_f7_tbot2021, avg_mean_upGVox_f8_tbot2021, avg_mean_upGVox_f9_tbot2021]);
avg_var_upGVox_all_tbot = 10^(-6) * mean([avg_var_upGVox_c0_tbot2021, avg_var_upGVox_f0_tbot2021, avg_var_upGVox_f1_tbot2021, avg_var_upGVox_f2_tbot2021, avg_var_upGVox_f3_tbot2021, avg_var_upGVox_f4_tbot2021, avg_var_upGVox_f5_tbot2021, avg_var_upGVox_f7_tbot2021, avg_var_upGVox_f8_tbot2021, avg_var_upGVox_f9_tbot2021]);

avg_mean_upHeur_all_tbot = 10^(-6) * mean([avg_mean_upHeur_c0_tbot2021, avg_mean_upHeur_f0_tbot2021, avg_mean_upHeur_f1_tbot2021, avg_mean_upHeur_f2_tbot2021, avg_mean_upHeur_f3_tbot2021, avg_mean_upHeur_f4_tbot2021, avg_mean_upHeur_f5_tbot2021, avg_mean_upHeur_f7_tbot2021, avg_mean_upHeur_f8_tbot2021, avg_mean_upHeur_f9_tbot2021]);
avg_var_upHeur_all_tbot = 10^(-6) * mean([avg_var_upHeur_c0_tbot2021, avg_var_upHeur_f0_tbot2021, avg_var_upHeur_f1_tbot2021, avg_var_upHeur_f2_tbot2021, avg_var_upHeur_f3_tbot2021, avg_var_upHeur_f4_tbot2021, avg_var_upHeur_f5_tbot2021, avg_var_upHeur_f7_tbot2021, avg_var_upHeur_f8_tbot2021, avg_var_upHeur_f9_tbot2021]);

avg_mean_selectT_all_tbot = 10^(-6) * mean([avg_mean_selectT_c0_tbot2021, avg_mean_selectT_f0_tbot2021, avg_mean_selectT_f1_tbot2021, avg_mean_selectT_f2_tbot2021, avg_mean_selectT_f3_tbot2021, avg_mean_selectT_f4_tbot2021, avg_mean_selectT_f5_tbot2021, avg_mean_selectT_f7_tbot2021, avg_mean_selectT_f8_tbot2021, avg_mean_selectT_f9_tbot2021]);
avg_var_selectT_all_tbot = 10^(-6) * mean([avg_var_selectT_c0_tbot2021, avg_var_selectT_f0_tbot2021, avg_var_selectT_f1_tbot2021, avg_var_selectT_f2_tbot2021, avg_var_selectT_f3_tbot2021, avg_var_selectT_f4_tbot2021, avg_var_selectT_f5_tbot2021, avg_var_selectT_f7_tbot2021, avg_var_selectT_f8_tbot2021, avg_var_selectT_f9_tbot2021]);

avg_mean_moveT_all_tbot = 10^(-6) * mean([avg_mean_moveT_c0_tbot2021, avg_mean_moveT_f0_tbot2021, avg_mean_moveT_f1_tbot2021, avg_mean_moveT_f2_tbot2021, avg_mean_moveT_f3_tbot2021, avg_mean_moveT_f4_tbot2021, avg_mean_moveT_f5_tbot2021, avg_mean_moveT_f7_tbot2021, avg_mean_moveT_f8_tbot2021, avg_mean_moveT_f9_tbot2021]);
avg_var_moveT_all_tbot = 10^(-6) * mean([avg_var_moveT_c0_tbot2021, avg_var_moveT_f0_tbot2021, avg_var_moveT_f1_tbot2021, avg_var_moveT_f2_tbot2021, avg_var_moveT_f3_tbot2021, avg_var_moveT_f4_tbot2021, avg_var_moveT_f5_tbot2021, avg_var_moveT_f7_tbot2021, avg_var_moveT_f8_tbot2021, avg_var_moveT_f9_tbot2021]);

total_process = avg_mean_upGVox_all_tbot + avg_mean_upHeur_all_tbot + avg_mean_selectT_all_tbot + avg_mean_moveT_all_tbot;

disp("During-navigation Average Computation Times: set1");
disp("Updating occupancy in the grid [ms]: Mean = " + avg_mean_upGVox_all_tbot + ", Var = " + avg_var_upGVox_all_tbot);
disp("Updating heuristics [ms]: Mean = " + avg_mean_upHeur_all_tbot + ", Var = " + avg_var_upHeur_all_tbot);
disp("Selecting the best tentacle [ms]: Mean = " + avg_mean_selectT_all_tbot + ", Var = " + avg_var_selectT_all_tbot);
disp("Moving the robot [ms]: Mean = " + avg_mean_moveT_all_tbot + ", Var = " + avg_var_moveT_all_tbot);
disp("Total (mean) [ms] = " + total_process);
disp(" ");

total_process = 10^(-6) * (avg_mean_upGVox_c0_set2_tbot + avg_mean_upHeur_c0_set2_tbot + avg_mean_selectT_c0_set2_tbot + avg_mean_moveT_c0_set2_tbot);

disp("During-navigation Average Computation Times: set2");
disp("Updating occupancy in the grid [ms]: Mean = " + 10^(-6) * avg_mean_upGVox_c0_set2_tbot + ", Var = " + 10^(-6) * avg_var_upGVox_c0_set2_tbot);
disp("Updating heuristics [ms]: Mean = " + 10^(-6) * avg_mean_upHeur_c0_set2_tbot + ", Var = " + 10^(-6) * avg_var_upHeur_c0_set2_tbot);
disp("Selecting the best tentacle [ms]: Mean = " + 10^(-6) * avg_mean_selectT_c0_set2_tbot + ", Var = " + 10^(-6) * avg_var_selectT_c0_set2_tbot);
disp("Moving the robot [ms]: Mean = " + 10^(-6) * avg_mean_moveT_c0_set2_tbot + ", Var = " + 10^(-6) * avg_var_moveT_c0_set2_tbot);
disp("Total (mean) [ms] = " + total_process);
disp(" ");

total_process = 10^(-6) * (avg_mean_upGVox_c0_set3_tbot + avg_mean_upHeur_c0_set3_tbot + avg_mean_selectT_c0_set3_tbot + avg_mean_moveT_c0_set3_tbot);

disp("During-navigation Average Computation Times: set3");
disp("Updating occupancy in the grid [ms]: Mean = " + 10^(-6) * avg_mean_upGVox_c0_set3_tbot + ", Var = " + 10^(-6) * avg_var_upGVox_c0_set3_tbot);
disp("Updating heuristics [ms]: Mean = " + 10^(-6) * avg_mean_upHeur_c0_set3_tbot + ", Var = " + 10^(-6) * avg_var_upHeur_c0_set3_tbot);
disp("Selecting the best tentacle [ms]: Mean = " + 10^(-6) * avg_mean_selectT_c0_set3_tbot + ", Var = " + 10^(-6) * avg_var_selectT_c0_set3_tbot);
disp("Moving the robot [ms]: Mean = " + 10^(-6) * avg_mean_moveT_c0_set3_tbot + ", Var = " + 10^(-6) * avg_var_moveT_c0_set3_tbot);
disp("Total (mean) [ms] = " + total_process);
disp(" ");

%% PLOT NAVIGATION SUCCESS
X = 1 : 10;
Y_success_ewokC7 = [success_rate_ewokC7_c0, success_rate_ewokC7_f0, success_rate_ewokC7_f1, success_rate_ewokC7_f2, success_rate_ewokC7_f3, success_rate_ewokC7_f4, success_rate_ewokC7_f5, success_rate_ewokC7_f7, success_rate_ewokC7_f8, success_rate_ewokC7_f9];
Y_success_ewokC9 = [success_rate_ewokC9_c0, success_rate_ewokC9_f0, success_rate_ewokC9_f1, success_rate_ewokC9_f2, success_rate_ewokC9_f3, success_rate_ewokC9_f4, success_rate_ewokC9_f5, success_rate_ewokC9_f7, success_rate_ewokC9_f8, success_rate_ewokC9_f9];
Y_success_tbot = [success_rate_tbot_c0, success_rate_tbot_f0, success_rate_tbot_f1, success_rate_tbot_f2, success_rate_tbot_f3, success_rate_tbot_f4, success_rate_tbot_f5, success_rate_tbot_f7, success_rate_tbot_f8, success_rate_tbot_f9];
Y_success_tbot2021 = [success_rate_tbot2021_c0, success_rate_tbot2021_f0, success_rate_tbot2021_f1, success_rate_tbot2021_f2, success_rate_tbot2021_f3, success_rate_tbot2021_f4, success_rate_tbot2021_f5, success_rate_tbot2021_f7, success_rate_tbot2021_f8, success_rate_tbot2021_f9];

Y_mean_success_ewokC7 = ones(1, 10) * mean(Y_success_ewokC7);
Y_mean_success_ewokC9 = ones(1, 10) * mean(Y_success_ewokC9);
Y_mean_success_tbot = ones(1, 10) * mean(Y_success_tbot);
Y_mean_success_tbot2021 = ones(1, 10) * mean(Y_success_tbot2021);

% figure;
% h1 = plot(X, Y_success_ewokC7, 'g', 'LineWidth', 2);
% hold on;
% h2 = plot(X, Y_success_ewokC9, 'b', 'LineWidth', 2);
% h3 = plot(X, Y_success_tbot, 'r', 'LineWidth', 2);
% h4 = plot(X, Y_mean_success_ewokC7, '--g', 'LineWidth', 3);
% h5 = plot(X, Y_mean_success_ewokC9, '--b', 'LineWidth', 3);
% h6 = plot(X, Y_mean_success_tbot, '--r', 'LineWidth', 3);
% 
% legend([h1, h2, h3], 'Usenko [1]: C=7', 'Usenko [1]: C=9', 'Tentabot');
% xlabel('Map name (C#: Cylinders, F#: Forest)');
% ylabel({'Navigation success rate';'(normalized)'});
% xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
% grid;
% hold off;

Y_success = [success_rate_ewokC7_c0, success_rate_ewokC9_c0, success_rate_tbot_c0, success_rate_tbot2021_c0; ...
                success_rate_ewokC7_f0, success_rate_ewokC9_f0, success_rate_tbot_f0, success_rate_tbot2021_f0; ...
                success_rate_ewokC7_f1, success_rate_ewokC9_f1, success_rate_tbot_f1, success_rate_tbot2021_f1; ...
                success_rate_ewokC7_f2, success_rate_ewokC9_f2, success_rate_tbot_f2, success_rate_tbot2021_f2; ...
                success_rate_ewokC7_f3, success_rate_ewokC9_f3, success_rate_tbot_f3, success_rate_tbot2021_f3; ...
                success_rate_ewokC7_f4, success_rate_ewokC9_f4, success_rate_tbot_f4, success_rate_tbot2021_f4; ...
                success_rate_ewokC7_f5, success_rate_ewokC9_f5, success_rate_tbot_f5, success_rate_tbot2021_f5; ...
                success_rate_ewokC7_f7, success_rate_ewokC9_f7, success_rate_tbot_f7, success_rate_tbot2021_f7; ...
                success_rate_ewokC7_f8, success_rate_ewokC9_f8, success_rate_tbot_f8, success_rate_tbot2021_f8; ...
                success_rate_ewokC7_f9, success_rate_ewokC9_f9, success_rate_tbot_f9, success_rate_tbot2021_f9;];

figure;
b = bar(Y_success);
b(1).FaceColor = [0 1 0];
b(2).FaceColor = [0 0 1];
b(3).FaceColor = [1 0 0];
b(4).FaceColor = [0 1 1];
hold on;
h4 = plot(X, Y_mean_success_ewokC7, '--g', 'LineWidth', 2);
h5 = plot(X, Y_mean_success_ewokC9, '--b', 'LineWidth', 2);
h6 = plot(X, Y_mean_success_tbot, '--r', 'LineWidth', 2);
h7 = plot(X, Y_mean_success_tbot2021, '--c', 'LineWidth', 2);

legend('Usenko [33]: C=7', 'Usenko [33]: C=9', 'Ours[1]', 'Ours');
xlabel('Map name (C#: Cylinders, F#: Forest)');
ylabel({'Navigation success rate';'(normalized)'});
xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
grid;
hold off;

%% PLOT NAVIGATION DURATION
Y_duration_ewokC7 = [mean_duration_ewokC7_c0, mean_duration_ewokC7_f0, mean_duration_ewokC7_f1, mean_duration_ewokC7_f2, mean_duration_ewokC7_f3, mean_duration_ewokC7_f4, mean_duration_ewokC7_f5, mean_duration_ewokC7_f7, mean_duration_ewokC7_f8, mean_duration_ewokC7_f9];
E_duration_ewokC7 = [var_duration_ewokC7_c0, var_duration_ewokC7_f0, var_duration_ewokC7_f1, var_duration_ewokC7_f2, var_duration_ewokC7_f3, var_duration_ewokC7_f4, var_duration_ewokC7_f5, var_duration_ewokC7_f7, var_duration_ewokC7_f8, var_duration_ewokC7_f9];
Y_duration_ewokC9 = [mean_duration_ewokC9_c0, mean_duration_ewokC9_f0, mean_duration_ewokC9_f1, mean_duration_ewokC9_f2, mean_duration_ewokC9_f3, mean_duration_ewokC9_f4, mean_duration_ewokC9_f5, mean_duration_ewokC9_f7, mean_duration_ewokC9_f8, mean_duration_ewokC9_f9];
E_duration_ewokC9 = [var_duration_ewokC9_c0, var_duration_ewokC9_f0, var_duration_ewokC9_f1, var_duration_ewokC9_f2, var_duration_ewokC9_f3, var_duration_ewokC9_f4, var_duration_ewokC9_f5, var_duration_ewokC9_f7, var_duration_ewokC9_f8, var_duration_ewokC9_f9];
Y_duration_tbot = [mean_duration_tbot_c0, mean_duration_tbot_f0, mean_duration_tbot_f1, mean_duration_tbot_f2, mean_duration_tbot_f3, mean_duration_tbot_f4, mean_duration_tbot_f5, mean_duration_tbot_f7, mean_duration_tbot_f8, mean_duration_tbot_f9];
E_duration_tbot = [var_duration_tbot_c0, var_duration_tbot_f0, var_duration_tbot_f1, var_duration_tbot_f2, var_duration_tbot_f3, var_duration_tbot_f4, var_duration_tbot_f5, var_duration_tbot_f7, var_duration_tbot_f8, var_duration_tbot_f9];
Y_duration_tbot2021 = [mean_duration_tbot2021_c0, mean_duration_tbot2021_f0, mean_duration_tbot2021_f1, mean_duration_tbot2021_f2, mean_duration_tbot2021_f3, mean_duration_tbot2021_f4, mean_duration_tbot2021_f5, mean_duration_tbot2021_f7, mean_duration_tbot2021_f8, mean_duration_tbot2021_f9];
E_duration_tbot2021 = [var_duration_tbot2021_c0, var_duration_tbot2021_f0, var_duration_tbot2021_f1, var_duration_tbot2021_f2, var_duration_tbot2021_f3, var_duration_tbot2021_f4, var_duration_tbot2021_f5, var_duration_tbot2021_f7, var_duration_tbot2021_f8, var_duration_tbot2021_f9];

%Y_mean_duration_ewokC7 = ones(1, 10) * mean(Y_duration_ewokC7);
Y_mean_duration_ewokC7 = ones(1, 10) * sum(Y_duration_ewokC7)/9;
%Y_mean_duration_ewokC9 = ones(1, 10) * mean(Y_duration_ewokC9);
Y_mean_duration_ewokC9 = ones(1, 10) * sum(Y_duration_ewokC9)/9;
Y_mean_duration_tbot = ones(1, 10) * mean(Y_duration_tbot);
Y_mean_duration_tbot2021 = ones(1, 10) * mean(Y_duration_tbot2021);

% figure;
% h1 = errorbar(X(1:5), Y_duration_ewokC7(1:5), E_duration_ewokC7(1:5), 'g', 'LineWidth', 2);
% hold on;
% h2 = errorbar(X(1:5), Y_duration_ewokC9(1:5), E_duration_ewokC9(1:5), 'b', 'LineWidth', 2);
% h3 = errorbar(X, Y_duration_tbot, E_duration_tbot, 'r', 'LineWidth', 2);
% h4 = plot(X, Y_mean_duration_ewokC7, '--g', 'LineWidth', 3);
% h5 = plot(X, Y_mean_duration_ewokC9, '--b', 'LineWidth', 3);
% h6 = plot(X, Y_mean_duration_tbot, '--r', 'LineWidth', 3);
% h7 = errorbar(X(7:end), Y_duration_ewokC7(7:end), E_duration_ewokC7(7:end), 'g', 'LineWidth', 2);
% h8 = errorbar(X(7:end), Y_duration_ewokC9(7:end), E_duration_ewokC9(7:end), 'b', 'LineWidth', 2);
% h1.LineStyle = 'none';
% h2.LineStyle = 'none';
% h3.LineStyle = 'none';
% 
% legend([h1, h2, h3], 'Usenko [1]: C=7', 'Usenko [1]: C=9', 'Tentabot');
% xlabel('Map name (C#: Cylinders, F#: Forest)');
% ylabel('Average navigation duration [s]');
% xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
% grid;
% hold off;

Y_duration = [mean_duration_ewokC7_c0, mean_duration_ewokC9_c0, mean_duration_tbot_c0, mean_duration_tbot2021_c0; ...
                mean_duration_ewokC7_f0, mean_duration_ewokC9_f0, mean_duration_tbot_f0, mean_duration_tbot2021_f0; ...
                mean_duration_ewokC7_f1, mean_duration_ewokC9_f1, mean_duration_tbot_f1, mean_duration_tbot2021_f1; ...
                mean_duration_ewokC7_f2, mean_duration_ewokC9_f2, mean_duration_tbot_f2, mean_duration_tbot2021_f2; ...
                mean_duration_ewokC7_f3, mean_duration_ewokC9_f3, mean_duration_tbot_f3, mean_duration_tbot2021_f3; ...
                mean_duration_ewokC7_f4, mean_duration_ewokC9_f4, mean_duration_tbot_f4, mean_duration_tbot2021_f4; ...
                mean_duration_ewokC7_f5, mean_duration_ewokC9_f5, mean_duration_tbot_f5, mean_duration_tbot2021_f5; ...
                mean_duration_ewokC7_f7, mean_duration_ewokC9_f7, mean_duration_tbot_f7, mean_duration_tbot2021_f7; ...
                mean_duration_ewokC7_f8, mean_duration_ewokC9_f8, mean_duration_tbot_f8, mean_duration_tbot2021_f8; ...
                mean_duration_ewokC7_f9, mean_duration_ewokC9_f9, mean_duration_tbot_f9, mean_duration_tbot2021_f9;];

X_err = [var_duration_ewokC7_c0, var_duration_ewokC9_c0, var_duration_tbot_c0, var_duration_tbot2021_c0; ...
                var_duration_ewokC7_f0, var_duration_ewokC9_f0, var_duration_tbot_f0, var_duration_tbot2021_f0; ...
                var_duration_ewokC7_f1, var_duration_ewokC9_f1, var_duration_tbot_f1, var_duration_tbot2021_f1; ...
                var_duration_ewokC7_f2, var_duration_ewokC9_f2, var_duration_tbot_f2, var_duration_tbot2021_f2; ...
                var_duration_ewokC7_f3, var_duration_ewokC9_f3, var_duration_tbot_f3, var_duration_tbot2021_f3; ...
                var_duration_ewokC7_f4, var_duration_ewokC9_f4, var_duration_tbot_f4, var_duration_tbot2021_f4; ...
                var_duration_ewokC7_f5, var_duration_ewokC9_f5, var_duration_tbot_f5, var_duration_tbot2021_f5; ...
                var_duration_ewokC7_f7, var_duration_ewokC9_f7, var_duration_tbot_f7, var_duration_tbot2021_f7; ...
                var_duration_ewokC7_f8, var_duration_ewokC9_f8, var_duration_tbot_f8, var_duration_tbot2021_f8; ...
                var_duration_ewokC7_f9, var_duration_ewokC9_f9, var_duration_tbot_f9, var_duration_tbot2021_f9;];         
            
figure;
b = bar(Y_duration);
b(1).FaceColor = [0 1 0];
b(2).FaceColor = [0 0 1];
b(3).FaceColor = [1 0 0];
b(4).FaceColor = [0 1 1];
hold on;

ngroups = size(Y_duration, 1);
nbars = size(Y_duration, 2);
% Calculating the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    er = errorbar(x, Y_duration(:,i), X_err(:,i), '.');
    er.Color = [0 0 0];                            
    er.LineStyle = 'none';
end

% h2 = errorbar(X(1:5), Y_duration_ewokC9(1:5), E_duration_ewokC9(1:5), 'b', 'LineWidth', 2);
% h3 = errorbar(X, Y_duration_tbot, E_duration_tbot, 'r', 'LineWidth', 2);
h4 = plot(X, Y_mean_duration_ewokC7, '--g', 'LineWidth', 2);
h5 = plot(X, Y_mean_duration_ewokC9, '--b', 'LineWidth', 2);
h6 = plot(X, Y_mean_duration_tbot, '--r', 'LineWidth', 2);
h7 = plot(X, Y_mean_duration_tbot2021, '--c', 'LineWidth', 2);
% h7 = errorbar(X(7:end), Y_duration_ewokC7(7:end), E_duration_ewokC7(7:end), 'g', 'LineWidth', 2);
% h8 = errorbar(X(7:end), Y_duration_ewokC9(7:end), E_duration_ewokC9(7:end), 'b', 'LineWidth', 2);

legend('Usenko [33]: C=7', 'Usenko [33]: C=9', 'Ours[1]', 'Ours');
xlabel('Map name (C#: Cylinders, F#: Forest)');
ylabel('Average navigation duration [s]');
xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
grid;
hold off;

%% PLOT NAVIGATION LENGTH
Y_length_ewokC7 = [mean_length_ewokC7_c0, mean_length_ewokC7_f0, mean_length_ewokC7_f1, mean_length_ewokC7_f2, mean_length_ewokC7_f3, mean_length_ewokC7_f4, mean_length_ewokC7_f5, mean_length_ewokC7_f7, mean_length_ewokC7_f8, mean_length_ewokC7_f9];
E_length_ewokC7 = [var_length_ewokC7_c0, var_length_ewokC7_f0, var_length_ewokC7_f1, var_length_ewokC7_f2, var_length_ewokC7_f3, var_length_ewokC7_f4, var_length_ewokC7_f5, var_length_ewokC7_f7, var_length_ewokC7_f8, var_length_ewokC7_f9];
Y_length_ewokC9 = [mean_length_ewokC9_c0, mean_length_ewokC9_f0, mean_length_ewokC9_f1, mean_length_ewokC9_f2, mean_length_ewokC9_f3, mean_length_ewokC9_f4, mean_length_ewokC9_f5, mean_length_ewokC9_f7, mean_length_ewokC9_f8, mean_length_ewokC9_f9];
E_length_ewokC9 = [var_length_ewokC9_c0, var_length_ewokC9_f0, var_length_ewokC9_f1, var_length_ewokC9_f2, var_length_ewokC9_f3, var_length_ewokC9_f4, var_length_ewokC9_f5, var_length_ewokC9_f7, var_length_ewokC9_f8, var_length_ewokC9_f9];
Y_length_tbot = [mean_length_tbot_c0, mean_length_tbot_f0, mean_length_tbot_f1, mean_length_tbot_f2, mean_length_tbot_f3, mean_length_tbot_f4, mean_length_tbot_f5, mean_length_tbot_f7, mean_length_tbot_f8, mean_length_tbot_f9];
E_length_tbot = [var_length_tbot_c0, var_length_tbot_f0, var_length_tbot_f1, var_length_tbot_f2, var_length_tbot_f3, var_length_tbot_f4, var_length_tbot_f5, var_length_tbot_f7, var_length_tbot_f8, var_length_tbot_f9];
Y_length_tbot2021 = [mean_length_tbot2021_c0, mean_length_tbot2021_f0, mean_length_tbot2021_f1, mean_length_tbot2021_f2, mean_length_tbot2021_f3, mean_length_tbot2021_f4, mean_length_tbot2021_f5, mean_length_tbot2021_f7, mean_length_tbot2021_f8, mean_length_tbot2021_f9];
E_length_tbot2021 = [var_length_tbot2021_c0, var_length_tbot2021_f0, var_length_tbot2021_f1, var_length_tbot2021_f2, var_length_tbot2021_f3, var_length_tbot2021_f4, var_length_tbot2021_f5, var_length_tbot2021_f7, var_length_tbot2021_f8, var_length_tbot2021_f9];

%Y_mean_length_ewokC7 = ones(1, 10) * mean(Y_length_ewokC7);
Y_mean_length_ewokC7 = ones(1, 10) * sum(Y_length_ewokC7)/9;
%Y_mean_length_ewokC9 = ones(1, 10) * mean(Y_length_ewokC9);
Y_mean_length_ewokC9 = ones(1, 10) * sum(Y_length_ewokC9)/9;
Y_mean_length_tbot = ones(1, 10) * mean(Y_length_tbot);
Y_mean_length_tbot2021 = ones(1, 10) * mean(Y_length_tbot2021);

% figure;
% h1 = errorbar(X(1:5), Y_length_ewokC7(1:5), E_length_ewokC7(1:5), 'g', 'LineWidth', 2);
% hold on;
% h2 = errorbar(X(1:5), Y_length_ewokC9(1:5), E_length_ewokC9(1:5), 'b', 'LineWidth', 2);
% h3 = errorbar(X, Y_length_tbot, E_length_tbot, 'r', 'LineWidth', 2);
% h4 = plot(X, Y_mean_length_ewokC7, '--g', 'LineWidth', 3);
% h5 = plot(X, Y_mean_length_ewokC9, '--b', 'LineWidth', 3);
% h6 = plot(X, Y_mean_length_tbot, '--r', 'LineWidth', 3);
% h7 = errorbar(X(7:end), Y_length_ewokC7(7:end), E_length_ewokC7(7:end), 'g', 'LineWidth', 2);
% h8 = errorbar(X(7:end), Y_length_ewokC9(7:end), E_length_ewokC9(7:end), 'b', 'LineWidth', 2);
% 
% legend([h1, h2, h3], 'Usenko [1]: C=7', 'Usenko [1]: C=9', 'Tentabot');
% xlabel('Map name (C#: Cylinders, F#: Forest)');
% ylabel('Average navigation length [m]');
% xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
% grid;
% hold off;

Y_length = [mean_length_ewokC7_c0, mean_length_ewokC9_c0, mean_length_tbot_c0, mean_length_tbot2021_c0; ...
                mean_length_ewokC7_f0, mean_length_ewokC9_f0, mean_length_tbot_f0, mean_length_tbot2021_f0; ...
                mean_length_ewokC7_f1, mean_length_ewokC9_f1, mean_length_tbot_f1, mean_length_tbot2021_f1; ...
                mean_length_ewokC7_f2, mean_length_ewokC9_f2, mean_length_tbot_f2, mean_length_tbot2021_f2; ...
                mean_length_ewokC7_f3, mean_length_ewokC9_f3, mean_length_tbot_f3, mean_length_tbot2021_f3; ...
                mean_length_ewokC7_f4, mean_length_ewokC9_f4, mean_length_tbot_f4, mean_length_tbot2021_f4; ...
                mean_length_ewokC7_f5, mean_length_ewokC9_f5, mean_length_tbot_f5, mean_length_tbot2021_f5; ...
                mean_length_ewokC7_f7, mean_length_ewokC9_f7, mean_length_tbot_f7, mean_length_tbot2021_f7; ...
                mean_length_ewokC7_f8, mean_length_ewokC9_f8, mean_length_tbot_f8, mean_length_tbot2021_f8; ...
                mean_length_ewokC7_f9, mean_length_ewokC9_f9, mean_length_tbot_f9, mean_length_tbot2021_f9;];

X_err = [var_duration_ewokC7_c0, var_duration_ewokC9_c0, var_duration_tbot_c0, var_duration_tbot2021_c0; ...
                var_length_ewokC7_f0, var_length_ewokC9_f0, var_length_tbot_f0, var_length_tbot2021_f0; ...
                var_length_ewokC7_f1, var_length_ewokC9_f1, var_length_tbot_f1, var_length_tbot2021_f1; ...
                var_length_ewokC7_f2, var_length_ewokC9_f2, var_length_tbot_f2, var_length_tbot2021_f2; ...
                var_length_ewokC7_f3, var_length_ewokC9_f3, var_length_tbot_f3, var_length_tbot2021_f3; ...
                var_length_ewokC7_f4, var_length_ewokC9_f4, var_length_tbot_f4, var_length_tbot2021_f4; ...
                var_length_ewokC7_f5, var_length_ewokC9_f5, var_length_tbot_f5, var_length_tbot2021_f5; ...
                var_length_ewokC7_f7, var_length_ewokC9_f7, var_length_tbot_f7, var_length_tbot2021_f7; ...
                var_length_ewokC7_f8, var_length_ewokC9_f8, var_length_tbot_f8, var_length_tbot2021_f8; ...
                var_length_ewokC7_f9, var_length_ewokC9_f9, var_length_tbot_f9, var_length_tbot2021_f9;]; 

figure;
b = bar(Y_length);
b(1).FaceColor = [0 1 0];
b(2).FaceColor = [0 0 1];
b(3).FaceColor = [1 0 0];
b(4).FaceColor = [0 1 1];
hold on;

ngroups = size(Y_length, 1);
nbars = size(Y_length, 2);
% Calculating the width for each bar group
groupwidth = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    x = (1:ngroups) - groupwidth/2 + (2*i-1) * groupwidth / (2*nbars);
    er = errorbar(x, Y_length(:,i), X_err(:,i), '.');
    er.Color = [0 0 0];                            
    er.LineStyle = 'none';
end

% h2 = errorbar(X(1:5), Y_duration_ewokC9(1:5), E_duration_ewokC9(1:5), 'b', 'LineWidth', 2);
% h3 = errorbar(X, Y_duration_tbot, E_duration_tbot, 'r', 'LineWidth', 2);
h4 = plot(X, Y_mean_length_ewokC7, '--g', 'LineWidth', 2);
h5 = plot(X, Y_mean_length_ewokC9, '--b', 'LineWidth', 2);
h6 = plot(X, Y_mean_length_tbot, '--r', 'LineWidth', 2);
h7 = plot(X, Y_mean_length_tbot2021, '--c', 'LineWidth', 2);
% h7 = errorbar(X(7:end), Y_duration_ewokC7(7:end), E_duration_ewokC7(7:end), 'g', 'LineWidth', 2);
% h8 = errorbar(X(7:end), Y_duration_ewokC9(7:end), E_duration_ewokC9(7:end), 'b', 'LineWidth', 2);

legend('Usenko [33]: C=7', 'Usenko [33]: C=9', 'Ours[1]', 'Ours');
xlabel('Map name (C#: Cylinders, F#: Forest)');
ylabel('Average navigation length [m]');
xticklabels({'C0', 'F0', 'F1', 'F2', 'F3', 'F4', 'F5', 'F7', 'F8', 'F9'})
grid;
hold off;