function Sensor_data_registrated = recalc_Posi(Sensor_data,beta,bias_attitude)
data_length = length(Sensor_data);

% ��������WGS����ϵ�µ�λ��
f = 1/298.257;
e_2 = 2*f - f^2;
a = 6378137;
data_length = length(Sensor_data);
for ii = 1 : data_length
    L_Sensor = Sensor_data(ii).Sensor_L;
    B_Sensor = Sensor_data(ii).Sensor_B;
    H_Sensor = Sensor_data(ii).Sensor_H;
    
    y = Sensor_data(ii).Sensor_Attitude(1) - bias_attitude(1); % ƫ����
    p = Sensor_data(ii).Sensor_Attitude(2) - bias_attitude(2); % ������
    r = Sensor_data(ii).Sensor_Attitude(3) - bias_attitude(3); % ��ת��
%     y = Sensor_data(ii).Sensor_Attitude(1); % ƫ����
%     p = Sensor_data(ii).Sensor_Attitude(2); % ������
%     r = Sensor_data(ii).Sensor_Attitude(3); % ��ת��
    % ����������Ĵ�ز�������ϵ��ֱ������ϵ�����״�ֱ������ϵ����ת����
    W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
                             cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
                             -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];    
    rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
    Radar_Position_WGS(1) = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
    Radar_Position_WGS(2) = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
    Radar_Position_WGS(3) = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);
    
    % 84�����������ת����
    H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
            cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
            0                 cos(B_Sensor)                     sin(B_Sensor)].';

    Tar_Theta = Sensor_data(ii).Tar_Theta - beta(1);
    Tar_Phi = Sensor_data(ii).Tar_Phi - beta(2);
    Tar_R = Sensor_data(ii).Tar_R - beta(3);
    
    [x_Meas,y_Meas,z_Meas] = sph2cart(Tar_Theta,Tar_Phi,Tar_R);
    x_T = [x_Meas;y_Meas;z_Meas];
    Tar_Posi = H1_b.' * W_From_Celiang_to_ENU * x_T + Radar_Position_WGS.';

    Sensor_data_registrated(ii).Tar_Theta = Tar_Theta;      % ��������ֵ
    Sensor_data_registrated(ii).Tar_Phi = Tar_Phi;
    Sensor_data_registrated(ii).Tar_R = Tar_R;

    Sensor_data_registrated(ii).Tar_Posi = Tar_Posi;
 
    Sensor_data_registrated(ii).Sensor_L = Sensor_data(ii).Sensor_L;
    Sensor_data_registrated(ii).Sensor_B = Sensor_data(ii).Sensor_B;
    Sensor_data_registrated(ii).Sensor_H = Sensor_data(ii).Sensor_H;
        
    Sensor_data_registrated(ii).Sensor_Attitude = [y;p;r]; % ������̬��
    
    Sensor_data_registrated(ii).Theta_Error = Sensor_data(ii).Theta_Error; 
    Sensor_data_registrated(ii).Phi_Error = Sensor_data(ii).Phi_Error; 
    Sensor_data_registrated(ii).Range_Error = Sensor_data(ii).Range_Error; 
    
%     Sensor_data_registrated(ii).Sensor_Type = Sensor_data(ii).Sensor_Type; 
    
end