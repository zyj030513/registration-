function Sensor_data_fusion = recalc_Posi_Fusion(Sensor_data,data_Fusion)
data_length = length(Sensor_data);

% 传感器在WGS坐标系下的位置
f = 1/298.257;
e_2 = 2*f - f^2;
a = 6378137;
data_length = length(Sensor_data);
for ii = 1 : data_length
    L_Sensor = Sensor_data(ii).Sensor_L;
    B_Sensor = Sensor_data(ii).Sensor_B;
    H_Sensor = Sensor_data(ii).Sensor_H;
    
    y = Sensor_data(ii).Sensor_Attitude(1); % 偏航角
    p = Sensor_data(ii).Sensor_Attitude(2); % 俯仰角
    r = Sensor_data(ii).Sensor_Attitude(3); % 滚转角
    
    % 传感器载体的大地测量坐标系（直角坐标系）到雷达直角坐标系的旋转矩阵
    W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
        cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
        -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];
    rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
    Radar_Position_WGS(1) = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
    Radar_Position_WGS(2) = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
    Radar_Position_WGS(3) = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);
    
    % 84到东北天的旋转矩阵
    H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
        cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
        0                 cos(B_Sensor)                     sin(B_Sensor)].';
    % 计算融合结果在测量坐标系下的值
    hx = W_From_Celiang_to_ENU.' * H1_b * (data_Fusion(ii,:) - Radar_Position_WGS).';
    Tar_R = sqrt(hx(1)^2 + hx(2)^2 + hx(3)^2);
    Tar_Theta = atan2(hx(2),hx(1));
    Tar_Phi = asin(hx(3)/Tar_R);
    
    Sensor_data_fusion(ii,1) = Tar_Theta;
    Sensor_data_fusion(ii,2) = Tar_Phi;
    Sensor_data_fusion(ii,3) = Tar_R;

    
%     Sensor_data_fusion(ii).Tar_Theta = Tar_Theta;      % 更新量测值
%     Sensor_data_fusion(ii).Tar_Phi = Tar_Phi;
%     Sensor_data_fusion(ii).Tar_R = Tar_R;
%     
%     Sensor_data_fusion(ii).Tar_Posi = Tar_Posi;
%     
%     Sensor_data_fusion(ii).Sensor_L = Sensor_data(ii).Sensor_L;
%     Sensor_data_fusion(ii).Sensor_B = Sensor_data(ii).Sensor_B;
%     Sensor_data_fusion(ii).Sensor_H = Sensor_data(ii).Sensor_H;
%     
%     Sensor_data_fusion(ii).Sensor_Attitude = [y;p;r]; % 更新姿态角
%     
%     Sensor_data_fusion(ii).Theta_Error = Sensor_data(ii).Theta_Error;
%     Sensor_data_fusion(ii).Phi_Error = Sensor_data(ii).Phi_Error;
%     Sensor_data_fusion(ii).Range_Error = Sensor_data(ii).Range_Error;
    
    %     Sensor_data_registrated(ii).Sensor_Type = Sensor_data(ii).Sensor_Type;
    
end