% 绘制LSQ数据的函数
function dENU_fig(data, mode) % mode为0绘制Fixed mode为1绘制Float mode为2时绘制SPP
    switch(mode)
        case 0
            % 提取数据
            Week = table2array(data(:, 1)); % 时间（周）
            Second = table2array(data(:, 2)); % 时间（秒）
            
            X = table2array(data(:, 3)); % 坐标
            Y = table2array(data(:, 4));
            Z = table2array(data(:, 5));

            dE = table2array(data(:, 6)); % 基线
            dN = table2array(data(:, 7));
            dU = table2array(data(:, 8));

            Ratio = table2array(data(:, 9)); % Ratio

            GPS = table2array(data(:, 10)); % 卫星
            BDS = table2array(data(:, 11));
        case 1
            Week = table2array(data(:, 1)); % 时间（周）
            Second = table2array(data(:, 2)); % 时间（秒）
            
            X = table2array(data(:, 3)); % 坐标
            Y = table2array(data(:, 4));
            Z = table2array(data(:, 5));

            dE = table2array(data(:, 6)); % 基线
            dN = table2array(data(:, 7));
            dU = table2array(data(:, 8));

            Sigma = table2array(data(:, 9)); % Sigma
            PDOP = table2array(data(:, 10)); % DOP值
            HDOP = table2array(data(:, 11));
            VDOP = table2array(data(:, 12));

            GPS = table2array(data(:, 13)); % 卫星
            BDS = table2array(data(:, 14));     
        case 2
            Week = table2array(data(:, 1)); % 时间（周）
            Second = table2array(data(:, 2)); % 时间（秒）
            
            X = table2array(data(:, 3)); % 坐标
            Y = table2array(data(:, 4));
            Z = table2array(data(:, 5));

            dE = table2array(data(:, 6)); % 基线
            dN = table2array(data(:, 7));
            dU = table2array(data(:, 8));
            
            PDOP = table2array(data(:,9)); % PDOP
            SigmaPos = table2array(data(:,10)); % SigmaPos
            
            BDS = table2array(data(:, 11)); % 卫星
            GPS = table2array(data(:, 12));   
    end
            % 求平均
            dE_mean = mean(dE); % 基线
            dN_mean = mean(dN);
            dU_mean = mean(dU);

            % 求标准差
            dE_std = std(dE);
            dN_std = std(dN);
            dU_std = std(dU);

            % 基线减去平均基线
            dE = dE - dE_mean;
            dN = dN - dN_mean;
            dU = dU - dU_mean;

            % 终端输出平均结果与标准差
            fprintf("Mean dENU: %.10fm %.10fm %.10fm\n", dE_mean, dN_mean, dU_mean);
            fprintf("Std dENU: %.10fm %.10fm %.10fm\n", dE_std, dN_std, dU_std);

            % 画图
            figure;

            % 计算周内秒
            time = Second; % 每周有604800秒

            % 设置时间范围
            time_min = min(time);
            time_max = max(time);

            % 上图：dE
            subplot(3, 1, 1);
            plot(time, dE, 'b', 'LineWidth', 1.5); % 时间为横轴，dE为纵轴
            ylabel('dE (m)');
            xlim([time_min, time_max]); % 设置横轴范围为周内秒
            grid on;

            % 中图：dN
            subplot(3, 1, 2);
            plot(time, dN, 'r', 'LineWidth', 1.5); % 时间为横轴，dN为纵轴
            ylabel('dN (m)');
            xlim([time_min, time_max]); % 设置横轴范围为周内秒
            grid on;

            % 下图：dU
            subplot(3, 1, 3);
            plot(time, dU, 'g', 'LineWidth', 1.5); % 时间为横轴，dU为纵轴
            xlabel('Time (Seconds)');
            ylabel('dU (m)');
            xlim([time_min, time_max]); % 设置横轴范围为周内秒
            grid on;

            % 画图
            figure;

            % 上图：GPS
            subplot(2, 1, 1);
            plot(time, GPS, 'r.', 'LineWidth', 1.5); % 时间为横轴，dN为纵轴
            ylabel('GPS (num)');
            xlim([time_min, time_max]); % 设置横轴范围为周内秒
            grid on;

            % 下图：BDS
            subplot(2, 1, 2);
            plot(time, BDS, 'g.', 'LineWidth', 1.5); % 时间为横轴，dU为纵轴
            xlabel('Time (Seconds)');
            ylabel('BDS (num)');
            xlim([time_min, time_max]); % 设置横轴范围为周内秒
            grid on;
        
end