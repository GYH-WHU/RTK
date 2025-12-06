function Ratio_fig(data)
    % 提取数据
    Week = table2array(data(:, 1));    % 时间（周）
    second = table2array(data(:, 2));  % 时间（秒）
    ratio = table2array(data(:, 6));   % Ratio列数据

    % 初始化固定解计数器
    FixedNum = 0;

    % 统计并输出固定解比例
    for i = 1:length(ratio)
        if ratio(i) >= 3
            FixedNum = FixedNum + 1;
        end
    end
    rate = FixedNum / length(ratio);  % 固定解的比例
    fprintf("Fixed rate: %.8f\n", rate);

    % 计算时间
    time = second;  % 每周有 604800 秒

    % 绘图
    figure;

    % 绘制 ratio >= 3 的点为蓝色
    plot(time(ratio >= 3), ratio(ratio >= 3), 'b.', 'LineWidth', 1.5);  % 蓝色点
    hold on;

    % 绘制 ratio < 3 的点为红色
    plot(time(ratio < 3), ratio(ratio < 3), 'r.', 'LineWidth', 1.5);  % 红色点

    % 绘制 ratio = 3 的水平线
    yline(3, 'k--', 'LineWidth', 1.5);  % 绘制水平虚线表示 ratio = 3

    % 设置图像标签
    xlabel('Time (Seconds)');
    ylabel('Ratio');
    grid on;

    % 设置横轴范围
    xlim([min(time), max(time)]);  % 确保横轴没有空白
end
