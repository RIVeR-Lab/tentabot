function [avg_mean_upGVox_data, avg_var_upGVox_data, avg_mean_upHeur_data, avg_var_upHeur_data, avg_mean_selectT_data, avg_var_selectT_data, avg_mean_moveT_data, avg_var_moveT_data] = tnav_bench_extract_process(path)

    files = dir(path + "*.csv");
    file_num = length(files);
        
    data_num = 10;
    
    mean_upGVox_data = zeros(1, data_num);
    var_upGVox_data = zeros(1, data_num);
    mean_upHeur_data = zeros(1, data_num);
    var_upHeur_data = zeros(1, data_num);
    mean_selectT_data = zeros(1, data_num);
    var_selectT_data = zeros(1, data_num);
    mean_moveT_data = zeros(1, data_num);
    var_moveT_data = zeros(1, data_num);

    c = 1;
    for k = 1 : file_num
        filename = char(path + files(k).name);

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
            
            mean_upGVox_data(1, c) = mean(upGVox_data);
            var_upGVox_data(1, c) = var(upGVox_data);
            
            mean_upHeur_data(1, c) = mean(upHeur_data);
            var_upHeur_data(1, c) = var(upHeur_data);
            
            mean_selectT_data(1, c) = mean(selectT_data);
            var_selectT_data(1, c) = var(selectT_data);
            
            mean_moveT_data(1, c) = mean(moveT_data);
            var_moveT_data(1, c) = var(moveT_data);
            
            c = c + 1;
        end
    end
    
    avg_mean_upGVox_data = mean(mean_upGVox_data);
    avg_var_upGVox_data = mean(var_upGVox_data);
    
    avg_mean_upHeur_data = mean(mean_upHeur_data);
    avg_var_upHeur_data = mean(var_upHeur_data);
    
    avg_mean_selectT_data = mean(mean_selectT_data);
    avg_var_selectT_data = mean(var_selectT_data);
    
    avg_mean_moveT_data = mean(mean_moveT_data);
    avg_var_moveT_data = mean(var_moveT_data);
end