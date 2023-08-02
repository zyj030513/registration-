function [Sensor_fusion_Filtered] = Kalman_filter2(data_Fusion,R_fusion_matrix,data_length)
T_fusion = 0.2;
Positon_Filtering = 1;
Filter_Init_Finished = 0;
% 计算过程噪声协方差矩阵
Q_matrix = Q_cal(T_fusion);
Phi = Phi_cal(T_fusion);
for ii = 1:data_length
    if Filter_Init_Finished == 0
        Position_Arrival_WGS = data_Fusion(ii,:);
        X_nn(Positon_Filtering,1:9) = zeros(1,9);
        X_nn(Positon_Filtering,1) = Position_Arrival_WGS(1);
        X_nn(Positon_Filtering,4) = Position_Arrival_WGS(2);
        X_nn(Positon_Filtering,7) = Position_Arrival_WGS(3);
        
        P_nn(Positon_Filtering,:,:) = zeros(9,9);
        P_nn(Positon_Filtering,1,1) = 1e0;
        P_nn(Positon_Filtering,2,2) = 1e-3;
        P_nn(Positon_Filtering,3,3) = 1e-6;
        P_nn(Positon_Filtering,4,4) = 1e0;
        P_nn(Positon_Filtering,5,5) = 1e-3;
        P_nn(Positon_Filtering,6,6) = 1e-6;
        P_nn(Positon_Filtering,7,7) = 1e0;
        P_nn(Positon_Filtering,8,8) = 1e-3;
        P_nn(Positon_Filtering,9,9) = 1e-6;
        P_nn = 1e6*P_nn;
        Filter_Init_Finished = 1;       % 完成滤波器初始化
        Filter_Status(Positon_Filtering) = 1;
    else
        % 计算H1的第一部分
        % 计算状态一步预测值
        X_n1n = Phi *  squeeze(X_nn(Positon_Filtering - 1,1:9)).';
        P_n1_n = Phi * squeeze(P_nn(Positon_Filtering - 1,:,:)) * Phi.' + Q_matrix;

        % 计算量测噪声协方差矩阵
%         R_matrix1 = squeeze(R_fusion_matrix(ii,:,:));
%         R_matrix = diag(diag(R_matrix1));
        R_matrix = squeeze(R_fusion_matrix(ii,:,:));
        H_matrix = [1 0 0 0 0 0 0 0 0;
                    0 0 0 1 0 0 0 0 0;
                    0 0 0 0 0 0 1 0 0];
        %% 更新目标状态
        K = P_n1_n * H_matrix.' * inv(H_matrix * P_n1_n * H_matrix.' + R_matrix);
        P_n1_n1 = (eye(9) - K * H_matrix) * P_n1_n;

        % 读取量测值
        Measurements(1) = data_Fusion(ii,1);
        Measurements(2) = data_Fusion(ii,2);
        Measurements(3) = data_Fusion(ii,3);
        X_n1n1 = X_n1n + K * (Measurements.' - H_matrix * X_n1n);
        
        Filter_Status(Positon_Filtering) = 1;   % 更新当前融合周期的滤波状态，标记为已更新
        X_nn(Positon_Filtering,:) = X_n1n1;
        P_nn(Positon_Filtering,:,:) = P_n1_n1;

    end
    Positon_Filtering = Positon_Filtering + 1;                   % 滤波计数加一
end

Sensor_fusion_Filtered.X_nn = X_nn;
Sensor_fusion_Filtered.P_nn = P_nn;
