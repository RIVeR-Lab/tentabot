function [mean_grid_data, var_grid_data, mean_tentacle_voxel_data, var_tentacle_voxel_data] = tnav_bench_extract_pre(path)

    files = dir(path + "*.csv");
    file_num = length(files);
        
    data_num = 10;
    
    grid_data = zeros(1, data_num);
    tentacle_voxel_data = zeros(1, data_num);

    c = 1;
    for k = 1 : file_num
        filename = char(path + files(k).name);

        if( contains(filename, "pre") )
            bench_data = csvimport(filename);
            grid_data(1, c) = bench_data{2,1};
            tentacle_voxel_data(1, c) = bench_data{2,2};
            c = c + 1;
        end
    end
    
    mean_grid_data = mean(grid_data);
    var_grid_data = var(grid_data);
    mean_tentacle_voxel_data = mean(tentacle_voxel_data);
    var_tentacle_voxel_data = var(tentacle_voxel_data);
end