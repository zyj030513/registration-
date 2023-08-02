function Hmatrix_Sensor = Hmatrix_calc_With_fusion_data_inv(data_Fusion,Sensor_data)
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

    hx = W_From_Celiang_to_ENU.' * H1_b * (data_Fusion(ii,:) - Radar_Position_WGS).';
    x1 = hx(1);
    y1 = hx(2);
    z1 = hx(3);
    %% 这里的H矩阵应该用x的预测值，而不是量测值
    H1_a11 = -y1/(x1^2+y1^2);
    H1_a12 = x1/(x1^2+y1^2);
    H1_a13 = 0;
    H1_a21 = -z1*x1/sqrt(x1^2+y1^2)/(x1^2+y1^2+z1^2);
    H1_a22 = -z1*y1/sqrt(x1^2+y1^2)/(x1^2+y1^2+z1^2);
    H1_a23 = (x1^2+y1^2)/sqrt(x1^2+y1^2)/(x1^2+y1^2+z1^2);
    H1_a31 = x1/sqrt(x1^2+y1^2+z1^2);
    H1_a32 = y1/sqrt(x1^2+y1^2+z1^2);
    H1_a33 = z1/sqrt(x1^2+y1^2+z1^2);
    
    H1_a = [H1_a11,H1_a12,H1_a13;
        H1_a21,H1_a22,H1_a23;
        H1_a31,H1_a32,H1_a33];
    
    % 计算H1
%     H_matrix1 = H1_a * H1_b;
    H_matrix1 = H1_a * W_From_Celiang_to_ENU.' * H1_b;
%     H_matrix = H_matrix1 * H_matrix2;
    %     H_matrix3 = eye(3);
    %     H_matrix = H_matrix3 * H_matrix1 * H_matrix2;
    Hmatrix_Sensor(ii).matrix = inv(H_matrix1);
end