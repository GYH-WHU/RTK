% 读取数据的函数
function data = ReadFile(filename, mode)% mode为0读取Fixed mode为1读取Float mode为2读取SPP mode为3读取RTKPlot文件
    fileID = fopen(filename, 'r');
    if fileID == -1
        error('无法打开文件');
    end
    switch(mode)
        case 0
            % 使用 textscan 读取文件数据
            data = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ' ', 'HeaderLines', 1);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2},data{3}, data{4},data{5}, data{6},data{7}, data{8},data{9}, data{10},data{11});
        case 1
            % 使用 textscan 读取文件数据
            data = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ' ', 'HeaderLines', 1);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2},data{3}, data{4},data{5}, data{6},data{7}, data{8},data{9}, data{10},data{11},data{12}, data{13},data{14}); 
        case 2
            % 使用 textscan 读取文件数据
            data = textscan(fileID, '%f %f %f %f %f %f %f %f %f %f %f %f', 'Delimiter', ' ', 'HeaderLines', 1);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2},data{3}, data{4},data{5},data{6}, data{7},data{8}, data{9},data{10},data{11},data{12}); 
        case 3
            % 使用 textscan 读取文件数据
            data = textscan(fileID, '%f %f %f %f %f %f %f', 'Delimiter', ' ', 'HeaderLines', 0);
                        
            % 将数据转为 table，列名可以根据需要进行修改
            data = table(data{1}, data{2},data{3}, data{4},data{5}, data{6},data{7});
            
    end
    % 关闭文件
    fclose(fileID);
end
