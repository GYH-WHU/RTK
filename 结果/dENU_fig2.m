% 绘制叠加图像的函数
function dENU_fig2(data1, data2, data3, mode)
    % 提取数据
    dE1 = table2array(data1(:, 6)); % 基线
    dN1 = table2array(data1(:, 7));
    dU1 = table2array(data1(:, 8));

    dE2 = table2array(data2(:, 6)); % 基线
    dN2 = table2array(data2(:, 7));
    dU2 = table2array(data2(:, 8));

    dE3 = table2array(data3(:, 6)); % 基线
    dN3 = table2array(data3(:, 7));
    dU3 = table2array(data3(:, 8));

    % 计算时间
    time1 = table2array(data1(:, 2)); % 时间（秒）
    time2 = table2array(data2(:, 2)); % 时间（秒）
    time3 = table2array(data3(:, 2)); % 时间（秒）

    % 获取最长时间长度
    time_min = min([min(time1), min(time2), min(time3)]);
    time_max = max([max(time1), max(time2), max(time3)]);

    % 使用相同的时间区间进行插值
    time = linspace(time_min, time_max, max([length(time1), length(time2), length(time3)]));

    % 插值
    dE1_interp = interp1(time1, dE1, time, 'linear', 'extrap');
    dN1_interp = interp1(time1, dN1, time, 'linear', 'extrap');
    dU1_interp = interp1(time1, dU1, time, 'linear', 'extrap');

    dE2_interp = interp1(time2, dE2, time, 'linear', 'extrap');
    dN2_interp = interp1(time2, dN2, time, 'linear', 'extrap');
    dU2_interp = interp1(time2, dU2, time, 'linear', 'extrap');

    dE3_interp = interp1(time3, dE3, time, 'linear', 'extrap');
    dN3_interp = interp1(time3, dN3, time, 'linear', 'extrap');
    dU3_interp = interp1(time3, dU3, time, 'linear', 'extrap');

    % 创建图像
    figure;

    switch mode
        case 0
            % 如果mode为0，绘制前两个数据的融合图

            % 上图：dE
            subplot(3, 1, 1);
            plot(time, dE2_interp, 'm', 'LineWidth', 1.5); hold on;
            plot(time, dE1_interp, 'g', 'LineWidth', 1.5);
            ylabel('dE (m)');
            legend('LSQ Float', 'LSQ Fixed');
            grid on;
            xlim([time_min, time_max]);  % 确保横轴没有空白

            % 中图：dN
            subplot(3, 1, 2);
            plot(time, dN2_interp, 'm', 'LineWidth', 1.5); hold on;
            plot(time, dN1_interp, 'g', 'LineWidth', 1.5);
            ylabel('dN (m)');
            grid on;
            xlim([time_min, time_max]);  % 确保横轴没有空白

            % 下图：dU
            subplot(3, 1, 3);
            plot(time, dU2_interp, 'm', 'LineWidth', 1.5); hold on;
            plot(time, dU1_interp, 'g', 'LineWidth', 1.5);
            xlabel('Time (Seconds)');
            ylabel('dU (m)');
            grid on;
            xlim([time_min, time_max]);  % 确保横轴没有空白

        case 1
            % 如果mode为1，绘制三个数据的融合图

            % 上图：dE
            subplot(3, 1, 1);
            plot(time, dE3_interp, 'b', 'LineWidth', 1.5); hold on;
            plot(time, dE2_interp, 'r', 'LineWidth', 1.5);
            plot(time, dE1_interp, 'g', 'LineWidth', 1.5);
            xlabel('Time (Seconds)');
            ylabel('dE (m)');
            legend('LSQ Fixed', 'LSQ Float', 'SPP');
            grid on;
            xlim([time_min, time_max]);

            % 中图：dN
            subplot(3, 1, 2);
            plot(time, dN3_interp, 'b', 'LineWidth', 1.5); hold on;
            plot(time, dN2_interp, 'r', 'LineWidth', 1.5);
            plot(time, dN1_interp, 'g', 'LineWidth', 1.5);
            xlabel('Time (Seconds)');
            ylabel('dN (m)');
            legend('LSQ Fixed', 'LSQ Float', 'SPP');
            grid on;
            xlim([time_min, time_max]);

            % 下图：dU
            subplot(3, 1, 3);
            plot(time, dU3_interp, 'b', 'LineWidth', 1.5); hold on;
            plot(time, dU2_interp, 'r', 'LineWidth', 1.5);
            plot(time, dU1_interp, 'g', 'LineWidth', 1.5);
            xlabel('Time (Seconds)');
            ylabel('dU (m)');
            legend('LSQ Fixed', 'LSQ Float', 'SPP');
            grid on;
            xlim([time_min, time_max]);
    end
end
