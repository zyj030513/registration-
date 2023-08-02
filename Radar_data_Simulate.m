% 2017.8.25 10:56   Radar_data_Simulate
% 说明：利用TargetForKF_E.m生成的目标飞行器点迹数据（84地心直角坐标系下，已保存为Target_Track.mat文件），模拟多个雷达站的目标飞行器观测数据。


    clc;
    close all;
    clear all;
    format long;


    load Target_Track.mat
    data_length = length(Ex);
 
%    %% 雷达站1位置      
%     L_rad_deg1 = 123.26;   % （外海距上海约216公里）
%     B_rad_deg1 = 32.47;
%     L_rad1 = L_rad_deg1*pi/180;  % 经度，单位：rad
%     B_rad1 = B_rad_deg1*pi/180;   % 纬度，单位：rad
%     H_rad1 = 10e3;             % 高度，单位：m
%     
%     
%    %% 雷达站2位置      
%     L_rad_deg2 = 123.30;   % （外海距上海约203公里）
%     B_rad_deg2 = 30.33;
%     L_rad2 = L_rad_deg2*pi/180;  % 经度，单位：rad
%     B_rad2 = B_rad_deg2*pi/180;   % 纬度，单位：rad
%     H_rad2 = 10e3;             % 高度，单位：m
    
    
    
   %% 雷达站1位置      
    L_rad_deg1 = 126.724577;   % 
    B_rad_deg1 = 33.357661;
    L_rad1 = L_rad_deg1*pi/180;  % 经度，单位：rad
    B_rad1 = B_rad_deg1*pi/180;   % 纬度，单位：rad
    H_rad1 = 0;             % 高度，单位：m
    
   %% 雷达站2位置      
    L_rad_deg2 = 130.514421;   % 
    B_rad_deg2 = 30.343170;
    L_rad2 = L_rad_deg2*pi/180;  % 经度，单位：rad
    B_rad2 = B_rad_deg2*pi/180;   % 纬度，单位：rad
    H_rad2 = 0;             % 高度，单位：m

    %% 坐标系转换矩阵设置
    % WGS-84地面点坐标系与WGS-84空间点坐标系之间转换的参数设置
    f = 1/298.257;
    e_2 = 2*f - f^2;
    a = 6378137;

    % 由WGS-84空间点坐标系到雷达1地理坐标系的转换矩阵
    W1 = [-sin(L_rad1)              cos(L_rad1)              0;...
          -cos(L_rad1)*sin(B_rad1) -sin(L_rad1)*sin(B_rad1)  cos(B_rad1);...
           cos(L_rad1)*cos(B_rad1)  sin(L_rad1)*cos(B_rad1)  sin(B_rad1)];

    % 由雷达1地理坐标系到WGS-84空间点坐标系的转换
    WT1 = [-sin(L_rad1) -cos(L_rad1)*sin(B_rad1) cos(L_rad1)*cos(B_rad1);...
          cos(L_rad1)  -sin(L_rad1)*sin(B_rad1) sin(L_rad1)*cos(B_rad1);...
          0            cos(B_rad1)              sin(B_rad1)];
      
    % 由WGS-84空间点坐标系到雷达2地理坐标系的转换矩阵
    W2 = [-sin(L_rad2)              cos(L_rad2)              0;...
          -cos(L_rad2)*sin(B_rad2) -sin(L_rad2)*sin(B_rad2)  cos(B_rad2);...
           cos(L_rad2)*cos(B_rad2)  sin(L_rad2)*cos(B_rad2)  sin(B_rad2)];

    % 由雷达2地理坐标系到WGS-84空间点坐标系的转换
    WT2 = [-sin(L_rad2) -cos(L_rad2)*sin(B_rad2) cos(L_rad2)*cos(B_rad2);...
          cos(L_rad2)  -sin(L_rad2)*sin(B_rad2) sin(L_rad2)*cos(B_rad2);...
          0             cos(B_rad2)             sin(B_rad2)];
    
    %% 载体姿态角
    alpha1_deg = 0;   % 航向角
    beta1_deg = 0;    % 俯仰角
    gama1_deg = 0;    % 横滚角
      
    alpha2_deg = 0;   % 航向角
    beta2_deg = 0;    % 俯仰角
    gama2_deg = 0;    % 横滚角
    
    alpha1 = alpha1_deg/180*pi;   % 航向角
    beta1 = beta1_deg/180*pi;    % 俯仰角
    gama1 = gama1_deg/180*pi;    % 横滚角
      
    alpha2 = alpha2_deg/180*pi;   % 航向角
    beta2 = beta2_deg/180*pi;    % 俯仰角
    gama2 = gama2_deg/180*pi;    % 横滚角
    % 由载体1地理坐标系（东北天，ENU1）到 雷达1直角坐标系的转换矩阵
    L1 = [cos(alpha1)*cos(gama1) + sin(alpha1)*sin(beta1)*sin(gama1)       -sin(alpha1)*cos(gama1) + cos(alpha1)*sin(beta1)*sin(gama1)      -cos(beta1)*sin(gama1);...
          sin(alpha1)*cos(beta1)                                            cos(alpha1)*cos(beta1)                                           sin(beta1);...
          cos(alpha1)*sin(gama1) - sin(alpha1)*sin(beta1)*cos(gama1)       -sin(alpha1)*sin(gama1) - cos(alpha1)*sin(beta1)*cos(gama1)       cos(beta1)*cos(gama1)
        ];

    % 由雷达1直角坐标系 到 载体地理坐标系（东北天，ENU）的转换
    LT1 = L1.';
    
    % 由载体2地理坐标系（东北天，ENU2）到 雷达2直角坐标系的转换矩阵
    L2 = [cos(alpha2)*cos(gama2) + sin(alpha2)*sin(beta2)*sin(gama2)       -sin(alpha2)*cos(gama2) + cos(alpha2)*sin(beta2)*sin(gama2)      -cos(beta2)*sin(gama2);...
          sin(alpha2)*cos(beta2)                                            cos(alpha2)*cos(beta2)                                           sin(beta2);...
          cos(alpha2)*sin(gama2) - sin(alpha2)*sin(beta2)*cos(gama2)       -sin(alpha2)*sin(gama2) - cos(alpha2)*sin(beta2)*cos(gama2)       cos(beta2)*cos(gama2)
        ];

    % 由雷达1直角坐标系 到 载体地理坐标系（东北天，ENU）的转换
    LT2 = L2.';
    %% 雷达站址在WGS-84空间点坐标系中的坐标
    %     雷达1 
    rew_rad1 = a / sqrt(1-e_2*(sin(B_rad1)^2));
    Exr1 = (rew_rad1 + H_rad1) * cos(B_rad1) * cos(L_rad1); 
    Eyr1 = (rew_rad1 + H_rad1) * cos(B_rad1) * sin(L_rad1);
    Ezr1 = (rew_rad1*(1-e_2) + H_rad1) * sin(B_rad1);
    
    %     雷达2 
    rew_rad2 = a / sqrt(1-e_2*(sin(B_rad2)^2));
    Exr2 = (rew_rad2 + H_rad2) * cos(B_rad2) * cos(L_rad2); 
    Eyr2 = (rew_rad2 + H_rad2) * cos(B_rad2) * sin(L_rad2);
    Ezr2 = (rew_rad2*(1-e_2) + H_rad2) * sin(B_rad2);
    
    
     %% 目标航迹由84空间点坐标系转换为雷达东北天坐标系
%     delt_Ex = M(1,1)*realx + M(1,2)*realy + M(1,3)*realz;
%     delt_Ey = M(2,1)*realx + M(2,2)*realy + M(2,3)*realz;
%     delt_Ez = M(3,1)*realx + M(3,2)*realy + M(3,3)*realz;     
    % 雷达1 
    delt_Ex = Ex - Exr1;
    delt_Ey = Ey - Eyr1;
    delt_Ez = Ez - Ezr1;
    
    ENU1_Ex = W1(1,1)*delt_Ex + W1(1,2)*delt_Ey + W1(1,3)*delt_Ez;
    ENU1_Ey = W1(2,1)*delt_Ex + W1(2,2)*delt_Ey + W1(2,3)*delt_Ez;
    ENU1_Ez = W1(3,1)*delt_Ex + W1(3,2)*delt_Ey + W1(3,3)*delt_Ez;   

    % 雷达2 
    delt_Ex = Ex - Exr2;
    delt_Ey = Ey - Eyr2;
    delt_Ez = Ez - Ezr2;
    
    ENU2_Ex = W2(1,1)*delt_Ex + W2(1,2)*delt_Ey + W2(1,3)*delt_Ez;
    ENU2_Ey = W2(2,1)*delt_Ex + W2(2,2)*delt_Ey + W2(2,3)*delt_Ez;
    ENU2_Ez = W2(3,1)*delt_Ex + W2(3,2)*delt_Ey + W2(3,3)*delt_Ez;   
    
    %% 目标航迹由雷达载体东北天坐标系转为阵面直角坐标系
    
    ARRAY1_Ex = L1(1,1)*ENU1_Ex + L1(1,2)*ENU1_Ey + L1(1,3)*ENU1_Ez;
    ARRAY1_Ey = L1(2,1)*ENU1_Ex + L1(2,2)*ENU1_Ey + L1(2,3)*ENU1_Ez;
    ARRAY1_Ez = L1(3,1)*ENU1_Ex + L1(3,2)*ENU1_Ey + L1(3,3)*ENU1_Ez;   

    ARRAY2_Ex = L2(1,1)*ENU2_Ex + L2(1,2)*ENU2_Ey + L2(1,3)*ENU2_Ez;
    ARRAY2_Ey = L2(2,1)*ENU2_Ex + L2(2,2)*ENU2_Ey + L2(2,3)*ENU2_Ez;
    ARRAY2_Ez = L2(3,1)*ENU2_Ex + L2(3,2)*ENU2_Ey + L2(3,3)*ENU2_Ez;
    
    ARRAY1_R = sqrt(ARRAY1_Ex.^2 +ARRAY1_Ey.^2 + ARRAY1_Ez.^2);   
    ARRAY1_El = asin(ARRAY1_Ez./ARRAY1_R);
    ARRAY1_Az = atan2(ARRAY1_Ex,ARRAY1_Ey);
    ARRAY1_El_deg = ARRAY1_El/pi*180;
    ARRAY1_Az_deg = ARRAY1_Az/pi*180;
    
    ARRAY2_R = sqrt(ARRAY2_Ex.^2 +ARRAY2_Ey.^2 + ARRAY2_Ez.^2);   
    ARRAY2_El = asin(ARRAY2_Ez./ARRAY2_R);
    ARRAY2_Az = atan2(ARRAY2_Ex,ARRAY2_Ey);
    ARRAY2_El_deg = ARRAY2_El/pi*180;
    ARRAY2_Az_deg = ARRAY2_Az/pi*180;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                     生成雷达观测数据                                     %   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %  距离、俯仰、方位噪声标准差
    sigma_R_1 = 10;        
    sigma_El_1 = 0.05;
    sigma_Az_1 = 0.05;
    sigma_R_2 = 10;        
    sigma_El_2 = 0.05;
    sigma_Az_2 = 0.05;
    
    R_bias_1 = -1000;        
    El_bias_1 = -1.5;
    Az_bias_1 = -1.5;
    R_bias_2 = 1000;        
    El_bias_2 = 1.5;
    Az_bias_2 = 1.5;
    
%     sigma_R_1 = 100;        
%     sigma_El_1 = 0.01;
%     sigma_Az_1 = 0.01;
%     sigma_R_2 = 100;        
%     sigma_El_2 = 0.01;
%     sigma_Az_2 = 0.01;
%     
%     R_bias_1 = -5000;        
%     El_bias_1 = -1;
%     Az_bias_1 = -1;
%     R_bias_2 = 5000;        
%     El_bias_2 = 1;
%     Az_bias_2 = 1;
%     
%     sigma_R_1 = 0;        
%     sigma_El_1 = 0;
%     sigma_Az_1 = 0;
%     
%     sigma_R_2 = 0;        
%     sigma_El_2 = 0;
%     sigma_Az_2 = 0;
    
%     R_bias_1 = 0;        
%     El_bias_1 = 0;
%     Az_bias_1 = 0;
%     R_bias_2 = 0;        
%     El_bias_2 = 0;
%     Az_bias_2 = 0;
    
    ARRAY1_R_measured = ARRAY1_R + sigma_R_1*randn(1,data_length) + R_bias_1;
    ARRAY1_El_deg_measured = ARRAY1_El_deg + sigma_El_1*randn(1,data_length) + El_bias_1;
    ARRAY1_Az_deg_measured = ARRAY1_Az_deg + sigma_Az_1*randn(1,data_length) + Az_bias_1;
    
    ARRAY2_R_measured = ARRAY2_R + sigma_R_2*randn(1,data_length) + R_bias_2;
    ARRAY2_El_deg_measured = ARRAY2_El_deg + sigma_El_2*randn(1,data_length) + El_bias_2;
    ARRAY2_Az_deg_measured = ARRAY2_Az_deg + sigma_Az_2*randn(1,data_length) + Az_bias_2;
    
% %     T= 0.1;
% %     timescale=[0:T:(data_length-1)*T];
% %     figure;
% %     plot(timescale,ARRAY1_R_measured/1e3)
% %     hold on 
% %     plot(timescale,ARRAY1_R/1e3,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('距离（Km）')
% %     title('雷达1距离量测')
% %     figure;
% %     plot(timescale,ARRAY1_El_deg_measured)
% %     hold on
% %     plot(timescale,ARRAY1_El_deg,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('俯仰角（°）')
% %     title('雷达1俯仰角量测')
% %     figure;
% %     plot(timescale,ARRAY1_Az_deg_measured)
% %     hold on
% %     plot(timescale,ARRAY1_Az_deg,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('方位角（°）')
% %     title('雷达2方位角量测')
% %     
% %     figure;
% %     plot(timescale,ARRAY2_R_measured/1e3)
% %     hold on 
% %     plot(timescale,ARRAY2_R/1e3,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('距离（Km）')
% %     title('雷达2距离量测')
% %     figure;
% %     plot(timescale,ARRAY2_El_deg_measured)
% %     hold on
% %     plot(timescale,ARRAY2_El_deg,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('俯仰角（°）')
% %     title('雷达2俯仰角量测')
% %     figure;
% %     plot(timescale,ARRAY2_Az_deg_measured)
% %      hold on
% %     plot(timescale,ARRAY2_Az_deg,'r','LineWidth',2)
% %     xlabel('时间（s）')
% %     ylabel('方位角（°）')
% %     title('雷达2方位角量测')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                                       空间配准                                          %   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 计算阵面直角坐标系量测
    ARRAY1_Ex_measured = ARRAY1_R_measured .* cos(ARRAY1_El_deg_measured/180*pi) .* sin(ARRAY1_Az_deg_measured/180*pi);
    ARRAY1_Ey_measured = ARRAY1_R_measured .* cos(ARRAY1_El_deg_measured/180*pi) .* cos(ARRAY1_Az_deg_measured/180*pi);
    ARRAY1_Ez_measured = ARRAY1_R_measured .* sin(ARRAY1_El_deg_measured/180*pi);
    
    ARRAY2_Ex_measured = ARRAY2_R_measured .* cos(ARRAY2_El_deg_measured/180*pi) .* sin(ARRAY2_Az_deg_measured/180*pi);
    ARRAY2_Ey_measured = ARRAY2_R_measured .* cos(ARRAY2_El_deg_measured/180*pi) .* cos(ARRAY2_Az_deg_measured/180*pi);
    ARRAY2_Ez_measured = ARRAY2_R_measured .* sin(ARRAY2_El_deg_measured/180*pi);
    
%     计算载体东北天坐标系下量测
    ENU1_Ex_measured = LT1(1,1)*ARRAY1_Ex_measured + LT1(1,2)*ARRAY1_Ey_measured + LT1(1,3)*ARRAY1_Ez_measured;
    ENU1_Ey_measured = LT1(2,1)*ARRAY1_Ex_measured + LT1(2,2)*ARRAY1_Ey_measured + LT1(2,3)*ARRAY1_Ez_measured;
    ENU1_Ez_measured = LT1(3,1)*ARRAY1_Ex_measured + LT1(3,2)*ARRAY1_Ey_measured + LT1(3,3)*ARRAY1_Ez_measured;

    ENU2_Ex_measured = LT2(1,1)*ARRAY2_Ex_measured + LT2(1,2)*ARRAY2_Ey_measured + LT2(1,3)*ARRAY2_Ez_measured;
    ENU2_Ey_measured = LT2(2,1)*ARRAY2_Ex_measured + LT2(2,2)*ARRAY2_Ey_measured + LT2(2,3)*ARRAY2_Ez_measured;
    ENU2_Ez_measured = LT2(3,1)*ARRAY2_Ex_measured + LT2(3,2)*ARRAY2_Ey_measured + LT2(3,3)*ARRAY2_Ez_measured;
    
%     计算两个雷达在84坐标系下的量测

    Ex1_measured = WT1(1,1)*ENU1_Ex_measured + WT1(1,2)*ENU1_Ey_measured + WT1(1,3)*ENU1_Ez_measured + Exr1;
    Ey1_measured = WT1(2,1)*ENU1_Ex_measured + WT1(2,2)*ENU1_Ey_measured + WT1(2,3)*ENU1_Ez_measured + Eyr1;
    Ez1_measured = WT1(3,1)*ENU1_Ex_measured + WT1(3,2)*ENU1_Ey_measured + WT1(3,3)*ENU1_Ez_measured + Ezr1;
    
    Ex2_measured = WT2(1,1)*ENU2_Ex_measured + WT2(1,2)*ENU2_Ey_measured + WT2(1,3)*ENU2_Ez_measured + Exr2;
    Ey2_measured = WT2(2,1)*ENU2_Ex_measured + WT2(2,2)*ENU2_Ey_measured + WT2(2,3)*ENU2_Ez_measured + Eyr2;
    Ez2_measured = WT2(3,1)*ENU2_Ex_measured + WT2(3,2)*ENU2_Ey_measured + WT2(3,3)*ENU2_Ez_measured + Ezr2;
    
% % 	T= 0.1;
% %     timescale=[0:T:(data_length-1)*T];
% %     figure;
% %     plot(timescale,Ex1_measured/1e3)
% %     hold on 
% %     plot(timescale,Ex2_measured/1e3,'r')
% %     hold on
% %     plot(timescale,Ex/1e3,'k','LineWidth',2)
% %     legend('雷达1量测','雷达2量测','真实航迹')
% %     xlabel('时间（s）')
% %     ylabel('Ex（Km）')
% %     title('Ex坐标数据')
% %     
% %     figure;
% %     plot(timescale,Ey1_measured/1e3)
% %     hold on 
% %     plot(timescale,Ey2_measured/1e3,'r')
% %     hold on
% %     plot(timescale,Ey/1e3,'k','LineWidth',2)
% %     legend('雷达1量测','雷达2量测','真实航迹')
% %     xlabel('时间（s）')
% %     ylabel('Ey（Km）')
% %     title('Ey坐标数据')
% %     
% %     figure;
% %     plot(timescale,Ez1_measured/1e3)
% %     hold on 
% %     plot(timescale,Ez2_measured/1e3,'r')
% %     hold on
% %     plot(timescale,Ez/1e3,'k','LineWidth',2)
% %     legend('雷达1量测','雷达2量测','真实航迹')
% %     xlabel('时间（s）')
% %     ylabel('Ez（Km）')
% %     title('Ez坐标数据')

    %% 空间配准
    % 方法一     有限记忆最小二乘
    % 参考论文   基于有限记忆最小二乘的雷达误差配准算法_米芳彬
    C = [];
    CT_invsigma_C = zeros(6,6);
    CT_invsigma_deltax = zeros(6,1);
%     for ii = 1 : data_length
%     for ii = 5001 : 9000
    for ii = 1 : 50 :10001
        Ln1 = [cosd(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii))     ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii));
               cosd(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii))    -ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii));
               sind(ARRAY1_El_deg_measured(ii))                                       ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii))                                        0];
        Jn1 = Ln1;
        
        Ln2 = [cosd(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii))     ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii));
               cosd(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii))    -ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii));
               sind(ARRAY2_El_deg_measured(ii))                                       ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii))                                       0];
        Jn2 = Ln2;
        
        Cn = [WT1 * Ln1 -WT2 * Ln2];
        C = [C; Cn];
        
        Hn = [WT1 * Jn1 -WT2 * Jn2];
        
        Sigma_n = Hn * diag([sigma_R_1,sigma_El_1*(pi/180)^2,sigma_Az_1*(pi/180)^2,sigma_R_2,sigma_El_2*(pi/180)^2,sigma_Az_2*(pi/180)^2]) * Hn.';
        
        CT_invsigma_C_n = Cn.' * inv(Sigma_n) * Cn;
%         CT_invsigma_C_n = Cn.' / Sigma_n * Cn;
        CT_invsigma_C = CT_invsigma_C + CT_invsigma_C_n;
        
        delta_measured = [Ex1_measured(ii) - Ex2_measured(ii);
                          Ey1_measured(ii) - Ey2_measured(ii);
                          Ez1_measured(ii) - Ez2_measured(ii);];
        CT_invsigma_deltax_n = Cn.' * inv(Sigma_n) * delta_measured;
%         CT_invsigma_deltax_n = Cn.' / Sigma_n * delta_measured;
        CT_invsigma_deltax = CT_invsigma_deltax + CT_invsigma_deltax_n;
    end
%     delta_measured = [Ex1_measured - Ex2_measured;
%                       Ey1_measured - Ey2_measured;
%                       Ez1_measured - Ez2_measured;];
    Estimate_bias_all = inv(CT_invsigma_C) * CT_invsigma_deltax;
%     Estimate_bias_all = CT_invsigma_C \ CT_invsigma_deltax
    Estimate_bias_all(2) = Estimate_bias_all(2)/pi*180;
    Estimate_bias_all(3) = Estimate_bias_all(3)/pi*180;
    Estimate_bias_all(5) = Estimate_bias_all(5)/pi*180;
    Estimate_bias_all(6) = Estimate_bias_all(6)/pi*180;
    Estimate_bias_all_LS = Estimate_bias_all
    
    % 方法二     Kalman滤波方法
    % 参考论文   基于高精度导航设备的海上多传感器配准算法_王成飞
    
    Xn_n = zeros(6,1);
%     Pn_n = [1e6  0    0    0    0    0;
%             0    1e-3 0    0    0    0;
%             0    0    1e-3 0    0    0;
%             0    0    0    1e6  0    0;
%             0    0    0    0    1e-3 0;
%             0    0    0    0    0    1e-3];
%     Pn_n = 1e0*Pn_n;
% 
%     Q = [1e6  0    0    0    0    0;
%          0    1e-3 0    0    0    0;
%          0    0    1e-3 0    0    0;
%          0    0    0    1e6  0    0;
%          0    0    0    0    1e-3 0;
%          0    0    0    0    0    1e-3];
%      Q = 1E3*Q;
    Pn_n = [1e3  0    0    0    0    0;
            0    1e-3 0    0    0    0;
            0    0    1e-3 0    0    0;
            0    0    0    1e3  0    0;
            0    0    0    0    1e-3 0;
            0    0    0    0    0    1e-3];
    Pn_n = 1e0*Pn_n;

    Q = [1e3  0    0    0    0    0;
         0    1e-3 0    0    0    0;
         0    0    1e-3 0    0    0;
         0    0    0    1e3  0    0;
         0    0    0    0    1e-3 0;
         0    0    0    0    0    1e-3];
     Q = 1E3*Q;

    Estimate_bias_all_Kalman = [];
	for ii = 1 : data_length
%     for ii = 1 : 5000
        Ln1 = [cosd(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii))     ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii));
               cosd(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(ii)) * cosd(ARRAY1_Az_deg_measured(ii))    -ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii)) * sind(ARRAY1_Az_deg_measured(ii));
               sind(ARRAY1_El_deg_measured(ii))                                       ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(ii))                                        0];
        Jn1 = Ln1;
        
        Ln2 = [cosd(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii))     ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii));
               cosd(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(ii)) * cosd(ARRAY2_Az_deg_measured(ii))    -ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii)) * sind(ARRAY2_Az_deg_measured(ii));
               sind(ARRAY2_El_deg_measured(ii))                                       ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(ii))                                       0];
        Jn2 = Ln2;
        
        Hn = [WT1 * Ln1 -WT2 * Ln2];              
        Mn = [WT1 * Jn1 -WT2 * Jn2];
        
        R = Mn * diag([sigma_R_1,sigma_El_1*(pi/180)^2,sigma_Az_1*(pi/180)^2,sigma_R_2,sigma_El_2*(pi/180)^2,sigma_Az_2*(pi/180)^2]) * Mn.';
        delta_measured = [Ex1_measured(ii) - Ex2_measured(ii);
                          Ey1_measured(ii) - Ey2_measured(ii);
                          Ez1_measured(ii) - Ez2_measured(ii);];
        Xn1_n0 = Xn_n;
        Pn1_n0 = Pn_n + Q;
        K = Pn1_n0 * Hn.' * inv(Hn * Pn1_n0 * Hn.' + R);
        Pn_n = (eye(6) - K * Hn) * Pn1_n0;
        Xn_n = Xn1_n0 + K * (delta_measured - Hn * Xn1_n0);
        
        Estimate_bias_all_Kalman = [Estimate_bias_all_Kalman Xn_n];
        Q_sum = 0;
        if ii > 100
            omiga = sum(Estimate_bias_all_Kalman(:,ii-19:ii).').'/20;
            for tt = ii - 19 : ii
               temp =  Estimate_bias_all_Kalman(:,tt) - omiga;
               Q1 = temp * temp.'/20;
               Q_sum = Q_sum + Q1;
            end
            Q = Q_sum;
        end
% %         CIV = 0;
% %         if ii > 100
% %             for tt = ii - 9 : ii
% %                delta_measured = [Ex1_measured(tt) - Ex2_measured(tt);
% %                                  Ey1_measured(tt) - Ey2_measured(tt);
% %                                  Ez1_measured(tt) - Ez2_measured(tt);];
% %                Ln1 = [cosd(ARRAY1_El_deg_measured(tt)) * sind(ARRAY1_Az_deg_measured(tt))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(tt)) * sind(ARRAY1_Az_deg_measured(tt))     ARRAY1_R_measured(tt) * cosd(ARRAY1_El_deg_measured(tt)) * cosd(ARRAY1_Az_deg_measured(tt));
% %                       cosd(ARRAY1_El_deg_measured(tt)) * cosd(ARRAY1_Az_deg_measured(tt))   -ARRAY1_R_measured(ii) * sind(ARRAY1_El_deg_measured(tt)) * cosd(ARRAY1_Az_deg_measured(tt))    -ARRAY1_R_measured(tt) * cosd(ARRAY1_El_deg_measured(tt)) * sind(ARRAY1_Az_deg_measured(tt));
% %                       sind(ARRAY1_El_deg_measured(tt))                                       ARRAY1_R_measured(ii) * cosd(ARRAY1_El_deg_measured(tt))                                        0];
% %         
% %                Ln2 = [cosd(ARRAY2_El_deg_measured(tt)) * sind(ARRAY2_Az_deg_measured(tt))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(tt)) * sind(ARRAY2_Az_deg_measured(tt))     ARRAY2_R_measured(tt) * cosd(ARRAY2_El_deg_measured(tt)) * cosd(ARRAY2_Az_deg_measured(tt));
% %                       cosd(ARRAY2_El_deg_measured(tt)) * cosd(ARRAY2_Az_deg_measured(tt))   -ARRAY2_R_measured(ii) * sind(ARRAY2_El_deg_measured(tt)) * cosd(ARRAY2_Az_deg_measured(tt))    -ARRAY2_R_measured(tt) * cosd(ARRAY2_El_deg_measured(tt)) * sind(ARRAY2_Az_deg_measured(tt));
% %                       sind(ARRAY2_El_deg_measured(tt))                                       ARRAY2_R_measured(ii) * cosd(ARRAY2_El_deg_measured(tt))                                       0];
% %         
% %                Hn = [WT1 * Ln1 -WT2 * Ln2];              
% %                temp =  delta_measured - Hn * Estimate_bias_all_Kalman(:,tt);
% %                CIV1 = temp * temp.'/10;
% %                CIV = CIV + CIV1;
% %             end
% %             Q = K * CIV * K.';
% %         end
    end

    figure;
    plot(Estimate_bias_all_Kalman(1,:))
    hold on
    plot(R_bias_1*ones(1,data_length),'r')
    
    figure;
    plot(Estimate_bias_all_Kalman(2,:)/pi*180)
    hold on
    plot(El_bias_1*ones(1,data_length),'r')
    
    figure;
    plot(Estimate_bias_all_Kalman(3,:)/pi*180)
    hold on
    plot(Az_bias_1*ones(1,data_length),'r')
    
    figure;
    plot(Estimate_bias_all_Kalman(4,:))
    hold on
    plot(R_bias_2*ones(1,data_length),'r')
    
    figure;
    plot(Estimate_bias_all_Kalman(5,:)/pi*180)
    hold on
    plot(El_bias_2*ones(1,data_length),'r')
    
    figure;
    plot(Estimate_bias_all_Kalman(6,:)/pi*180)
    hold on
    plot(Az_bias_2*ones(1,data_length),'r')
    