function [data_filtered] = Kalman_filter(Data_input,Data_length)
T_fusion = 0.2;
Positon_Filtering = 1;
Filter_Init_Finished = 0;
% �����������Э�������
Q_matrix = Q_cal(T_fusion);
Phi = Phi_cal(T_fusion);
for ii = 1:Data_length
    if Filter_Init_Finished == 0
        Position_Arrival_WGS = Calc_Position_WGS(Data_input(ii));
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
        Filter_Init_Finished = 1;       % ����˲�����ʼ��
        Filter_Status(Positon_Filtering) = 1;
    else
        L_Sensor = Data_input(ii).Sensor_L;
        B_Sensor = Data_input(ii).Sensor_B;
        H_Sensor = Data_input(ii).Sensor_H;
        % ��������WGS����ϵ�µ�λ��
        f = 1/298.257;
        e_2 = 2*f - f^2;
        a = 6378137;
        rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
        Radar_Position_WGS(1) = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
        Radar_Position_WGS(2) = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
        Radar_Position_WGS(3) = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);
        
        % 84�����������ת����
        H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
                cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
                0                 cos(B_Sensor)                     sin(B_Sensor)].';
        
        % �����������
        % ��̬����
        % �����������ĸ����ǡ���ת�ǡ�ƫ����
        p = Data_input(ii).Sensor_Attitude(2); % ������
        r = Data_input(ii).Sensor_Attitude(3); % ��ת��
        y = Data_input(ii).Sensor_Attitude(1); % ƫ����
        
        % ����������Ĵ�ز�������ϵ�������죩���״�ֱ������ϵ����ת����
        W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
                                 cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
                                 -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];
        
        W_From_ENU_to_Celiang = W_From_Celiang_to_ENU.';
        % ����H1�ĵ�һ����
        % ����״̬һ��Ԥ��ֵ
        X_n1n = Phi *  squeeze(X_nn(Positon_Filtering - 1,1:9)).';
        P_n1_n = Phi * squeeze(P_nn(Positon_Filtering - 1,:,:)) * Phi.' + Q_matrix;
        
        % X_n1n1 = X_n1n + K * (Z.' - H_matrix * X_n1n);
        hx = W_From_ENU_to_Celiang * H1_b * ([X_n1n(1) X_n1n(4) X_n1n(7)] - Radar_Position_WGS).';
        
        x1 = hx(1);
        y1 = hx(2);
        z1 = hx(3);
        %% �����H����Ӧ����x��Ԥ��ֵ������������ֵ
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
        
        % ����H1
        H_matrix1 = H1_a * W_From_ENU_to_Celiang * H1_b;
        H_matrix2 = [1 0 0 0 0 0 0 0 0;
            0 0 0 1 0 0 0 0 0;
            0 0 0 0 0 0 1 0 0];
        H_matrix3 = eye(3);
        H_matrix = H_matrix3 * H_matrix1 * H_matrix2;
        
        % ������������Э�������
        R_matrix = [(Data_input(ii).Theta_Error)^2   0                               0;
            0                           (Data_input(ii).Phi_Error)^2         0;
            0                           0                               (Data_input(ii).Range_Error)^2];
        
        
        
        %% ����Ŀ��״̬
        K = P_n1_n * H_matrix.' * inv(H_matrix * P_n1_n * H_matrix.' + R_matrix);
        P_n1_n1 = (eye(9) - K * H_matrix) * P_n1_n;
        range = sqrt(hx(1)^2 + hx(2)^2 + hx(3)^2);
        az = atan2(hx(2),hx(1));
        el = asin(hx(3)/range);
        % ��ȡ����ֵ
        Measurements(1) = Data_input(ii).Tar_Theta;
        Measurements(2) = Data_input(ii).Tar_Phi;
        Measurements(3) = Data_input(ii).Tar_R;
        X_n1n1 = X_n1n + K * (Measurements.' - [az;el;range]);
        
        Filter_Status(Positon_Filtering) = 1;   % ���µ�ǰ�ں����ڵ��˲�״̬�����Ϊ�Ѹ���
        X_nn(Positon_Filtering,:) = X_n1n1;
        P_nn(Positon_Filtering,:,:) = P_n1_n1;
        
        
    end
    Positon_Filtering = Positon_Filtering + 1;                   % �˲�������һ
end


data_filtered.X_nn = X_nn;
data_filtered.P_nn = P_nn;


end