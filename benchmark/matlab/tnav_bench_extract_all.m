function [grid_data, tentacle_voxel_data, avg_upGVox_data, avg_upHeur_data, avg_selectT_data, avg_moveT_data, success_data, duration_data, length_data] = tnav_bench_extract_all(path)

    files = dir(path + "*.csv");
    file_num = length(files);
    
    data_num = 0.25 * file_num;
    
    grid_data = zeros(1, data_num);
    tentacle_voxel_data = zeros(1, data_num);
    
    avg_upGVox_data = zeros(1, data_num);
    avg_upHeur_data = zeros(1, data_num);
    avg_selectT_data = zeros(1, data_num);
    avg_moveT_data = zeros(1, data_num);
    
    success_data = zeros(1, data_num);
    duration_data = zeros(1, data_num);
    length_data = zeros(1, data_num);

    c1 = 1;
    c2 = 1;
    c3 = 1;
    for k = 1 : file_num
        filename = char(path + files(k).name);

        if( contains(filename, "pre") )
            bench_data = csvimport(filename);
            grid_data(1, c1) = bench_data{2,1};
            tentacle_voxel_data(1, c1) = bench_data{2,2};
            c1 = c1 + 1;
        end
        
        if( contains(filename, "process") )
            bench_data = csvimport(filename);
            dsize = size(bench_data, 1);
            upGVox_data = zeros(1, dsize);
            upHeur_data = zeros(1, dsize);
            selectT_data = zeros(1, dsize);
            moveT_data = zeros(1, dsize);
            for i = 2: dsize
                upGVox_data(1, i) = bench_data{i,1};
                upHeur_data(1, i) = bench_data{i,2};
                selectT_data(1, i) = bench_data{i,3};
                moveT_data(1, i) = bench_data{i,4};
            end
            avg_upGVox_data(1, c2) = mean(upGVox_data);
            avg_upHeur_data(1, c2) = mean(upHeur_data);
            avg_selectT_data(1, c2) = mean(selectT_data);
            avg_moveT_data(1, c2) = mean(moveT_data);
            c2 = c2 + 1;
        end
        
        if( contains(filename, "result") )
            bench_data = csvimport(filename);
            success_data(1, c3) = bench_data{2,1};
            duration_data(1, c3) = bench_data{2,2};
            length_data(1, c3) = bench_data{2,3};
            c3 = c3 + 1;
        end
    
    end 

end

