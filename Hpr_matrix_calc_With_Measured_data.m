function Hmatrix_Sensor = Hpr_matrix_calc_With_Measured_data(Sensor_data)
data_length = length(Sensor_data);

% 传感器在WGS坐标系下的位置
f = 1/298.257;
e_2 = 2*f - f^2;
a = 6378137;
for ii = 1 : data_length
    L_Sensor = Sensor_data(ii).Sensor_L;
    B_Sensor = Sensor_data(ii).Sensor_B;
    H_Sensor = Sensor_data(ii).Sensor_H;
    
    rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
    Radar_Position_WGS(1) = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
    Radar_Position_WGS(2) = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
    Radar_Position_WGS(3) = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);
    
    % 84到东北天的旋转矩阵
    H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
            cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
            0                 cos(B_Sensor)                     sin(B_Sensor)].';
    
    % 己方传感器的俯仰角、滚转角、偏航角
    p = Sensor_data(ii).Sensor_Attitude(2); % 俯仰角
    r = Sensor_data(ii).Sensor_Attitude(3); % 滚转角
    y = Sensor_data(ii).Sensor_Attitude(1); % 偏航角
     % 传感器载体的大地测量坐标系（直角坐标系）到雷达直角坐标系的旋转矩阵
    W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
                             cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
                             -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];
    hx = W_From_Celiang_to_ENU.' * H1_b * ( Sensor_data(ii).Tar_Posi - Radar_Position_WGS.');
    
    Hp1 = [-sin(r)*cos(p)*sin(y)    sin(p)*sin(y)	cos(r)*cos(p)*sin(y);
          sin(r)*cos(p)*cos(y)     -sin(p)*cos(y)   -cos(r)*cos(p)*cos(y);
          sin(r)*sin(p)            cos(p)           -cos(r)*sin(p)];
%     Hy1 = [-cos(r)*sin(y)-sin(r)*sin(p)*cos(y)    -cos(p)*cos(y)   	-sin(r)*sin(y)+cos(r)*sin(p)*cos(y);
%           cos(r)*cos(y)-sin(r)*sin(p)*sin(y)     -cos(p)*sin(y)     sin(r)*cos(y)+cos(r)*sin(p)*sin(y);
%           0                                      0                  0];
    Hr1 = [-sin(r)*cos(y)-cos(r)*sin(p)*sin(y)    0   	cos(r)*cos(y)-sin(r)*sin(p)*sin(y);
           -sin(r)*sin(y)+cos(r)*sin(p)*cos(y)    0     cos(r)*sin(y)+sin(r)*sin(p)*cos(y);
           -cos(r)*cos(p)                         0     -sin(r)*cos(p)];
      
    Hp = H1_b.' * Hp1 * hx;
    Hr = H1_b.' * Hr1 * hx;
    Hmatrix_Sensor(ii).matrix = [Hp Hr];
%     Hy = H1_b.' * Hy1 * hx;
%     Hmatrix_Sensor(ii).matrix = [Hy Hp Hr];
end