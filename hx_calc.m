function hx = hx_calc(X_nn_EKF_all,Frame_Now)

L_Sensor = Frame_Now.Sensor_L;
B_Sensor = Frame_Now.Sensor_B;
H_Sensor = Frame_Now.Sensor_H;
% 传感器在WGS坐标系下的位置
f = 1/298.257;
e_2 = 2*f - f^2;
a = 6378137;
rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
Radar_Position_WGS(1) = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
Radar_Position_WGS(2) = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
Radar_Position_WGS(3) = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);

% 84到东北天的旋转矩阵
H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
        cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
        0                 cos(B_Sensor)                     sin(B_Sensor)].';

% 计算量测矩阵
% 姿态矩阵
% 己方传感器的俯仰角、滚转角、偏航角
p = Frame_Now.Sensor_Attitude(2); % 俯仰角
r = Frame_Now.Sensor_Attitude(3); % 滚转角
y = Frame_Now.Sensor_Attitude(1); % 偏航角

% 传感器载体的大地测量坐标系（东北天）到雷达直角坐标系的旋转矩阵
W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
                         cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
                         -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];

W_From_ENU_to_Celiang = W_From_Celiang_to_ENU.';
Temp = W_From_ENU_to_Celiang * H1_b * ([X_nn_EKF_all(1) X_nn_EKF_all(2) X_nn_EKF_all(3)] - Radar_Position_WGS).';
hx(3) = sqrt(Temp(1)^2 + Temp(2)^2 + Temp(3)^2);
hx(1) = atan2(Temp(2),Temp(1));
hx(2) = asin(Temp(3)/hx(3));