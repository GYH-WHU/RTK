% main.m
% 提示用户输入
choice = input('请选择: 0 LSQ 浮点解/固定解对比; 1 SPP/EKF/LSQ对比; 2 分析Ratio：');
if(~(choice == 0 || choice == 1 || choice == 2))
    fprintf("\n无法识别：%d 程序结束!\n",choice);
end
switch(choice)
    case 0
        FILE_LSQ_Fixed = "LSQ/10.31（晚六小时）/FIXED_LSQ_FILE.txt";
        FILE_LSQ_Float = "LSQ/10.31（晚六小时）/FLOAT_LSQ_FILE.txt";
        
        LSQ_FIXED_Dat = ReadFile(FILE_LSQ_Fixed,0);
        LSQ_FLOAT_Dat = ReadFile(FILE_LSQ_Float,1);
        
        dENU_fig(LSQ_FIXED_Dat, 0);
        dENU_fig(LSQ_FLOAT_Dat, 1);
        % 绘制叠加图
        dENU_fig2(LSQ_FIXED_Dat, LSQ_FLOAT_Dat, LSQ_FLOAT_Dat, 0);
    case 1
        FILE_EKF = "EKF/10.31（晚六小时）/EKF_FILE.txt";
        FILE_LSQ_Fixed = "LSQ/10.31（晚六小时）/FIXED_LSQ_FILE.txt";
        FILE_SPP ="LSQ/10.31（晚六小时）/SPP_FILE_1.txt";
        
        EKF_Dat = ReadFile(FILE_EKF,0);
        LSQ_FIXED_Dat = ReadFile(FILE_LSQ_Fixed,0);
        SPP_Dat = ReadFile(FILE_SPP,2);        

        dENU_fig(EKF_Dat,0);
        dENU_fig(LSQ_FIXED_Dat, 0);
        dENU_fig(SPP_Dat,2);  
        % 绘制叠加图
        dENU_fig2(EKF_Dat, LSQ_FIXED_Dat, SPP_Dat, 0);

    case 2
        FILE_EKF_RTKPlot = "EKF/10.31（晚六小时）/RTKPlot.pos";
        FILE_LSQ_RTKPlot = "LSQ/10.31（晚六小时）/RTKPlot.pos";

        EKF_RTK_Dat = ReadFile(FILE_EKF_RTKPlot,3);
        LSQ_RTK_Dat = ReadFile(FILE_LSQ_RTKPlot,3);

        fprintf("EKF result: \n");
        Ratio_fig(EKF_RTK_Dat); % 绘图
        fprintf("LSQ result: \n");
        Ratio_fig(LSQ_RTK_Dat); % 绘图
        
end