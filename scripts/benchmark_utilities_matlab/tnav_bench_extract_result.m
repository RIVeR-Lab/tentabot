function [success_rate, mean_duration, var_duration, mean_length, var_length] = tnav_bench_extract_result(path)

    files = dir(path + "*.csv");
    file_num = length(files);
        
    data_num = 10;
    
    success_data = zeros(1, data_num);
    duration_data = zeros(1, 1);
    length_data = zeros(1, 1);

    cs = 1;
    co = 1;
    for k = 1 : file_num
        filename = char(path + files(k).name);

        if( contains(filename, "result") || contains(filename, "ewok") )
            bench_data = csvimport(filename);

            if(bench_data{2,1} == 1)
                success_data(1, cs) = 1;
            else
                success_data(1, cs) = 0;
            end

            if(success_data(1, cs) == 1) 
                duration_data(1, co) = bench_data{2,2};
                length_data(1, co) = bench_data{2,3};
                co = co + 1;
            end
            cs = cs + 1; 
        end
    end
    
    success_rate = mean(success_data);
    
    if(size(duration_data, 2) == 1)
        mean_duration = 0;   
        var_duration = 0;
    else
        mean_duration = mean(duration_data);   
        var_duration = var(duration_data);
    end
    
    if(size(duration_data, 2) == 1)
        mean_length = 0;   
        var_length = 0;
    else
        mean_length = mean(length_data);
        var_length = var(length_data);
    end
end
