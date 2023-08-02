function Position_Arrival_WGS = Calc_Position_WGS(Frame)
    % �״����ֱ������ϵ
    Position_Arrival_Polar(1) = Frame.Tar_Theta;                                     % �״�������ϵ�·�λ��
    Position_Arrival_Polar(2) = Frame.Tar_Phi;                                       % �״�������ϵ�¸�����
    Position_Arrival_Polar(3) = Frame.Tar_R;                                         % �״�������ϵ��б��
    Position_Arrival_Rect(1) = Position_Arrival_Polar(3) * cos(Position_Arrival_Polar(2)) * cos(Position_Arrival_Polar(1));   % �����״�ֱ������ϵ������ֵ
    Position_Arrival_Rect(2) = Position_Arrival_Polar(3) * cos(Position_Arrival_Polar(2)) * sin(Position_Arrival_Polar(1));
    Position_Arrival_Rect(3) = Position_Arrival_Polar(3) * sin(Position_Arrival_Polar(2));
    % �����״�����λ�ö���������ϵ��Ŀ��λ��
    % �����������ĸ����ǡ���ת�ǡ�ƫ����
    p = Frame.Sensor_Attitude(2); % ������
    r = Frame.Sensor_Attitude(3); % ��ת��
    y = Frame.Sensor_Attitude(1); % ƫ����

    % ����������Ĵ�ز�������ϵ��ֱ������ϵ�����״�ֱ������ϵ����ת����
    W_From_Celiang_to_ENU = [cos(r)*cos(y)-sin(r)*sin(p)*sin(y)    -cos(p)*sin(y)	sin(r)*cos(y) + cos(r)*sin(p)*sin(y);
                             cos(r)*sin(y)+sin(r)*sin(p)*cos(y)    cos(p)*cos(y)	sin(r)*sin(y) - cos(r)*sin(p)*cos(y);
                             -sin(r)*cos(p)                        sin(p)           cos(r)*cos(p)];
    Position_Arrival_ENU = W_From_Celiang_to_ENU * Position_Arrival_Rect.';
    % ����WGS84����ϵ��Ŀ��λ������(�����쵽WGS84)
%     Radar_ID = Frame_recieved(Frame_Recieved_Now).Radar_ID;   
%     Position_Arrival_WGS = Coordinate_Transformation_Radar_to_84(Position_Arrival_Rect,Radar_Position_WGS,Trasmint_From_Radar_ENU_to_84,Radar_ID);
    L_Sensor = Frame.Sensor_L;
    B_Sensor = Frame.Sensor_B;
    H_Sensor = Frame.Sensor_H;
    f = 1/298.257;
    e_2 = 2*f - f^2;
    a = 6378137;
    rew_rad = a / sqrt(1-e_2*(sin(B_Sensor)^2));
    Exr = (rew_rad + H_Sensor) * cos(B_Sensor) * cos(L_Sensor);
    Eyr = (rew_rad + H_Sensor) * cos(B_Sensor) * sin(L_Sensor);
    Ezr = (rew_rad*(1-e_2) + H_Sensor) * sin(B_Sensor);
	Trasmint_From_Radar_ENU_to_84_1 = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
                                       cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
                                       0                 cos(B_Sensor)                     sin(B_Sensor)];
	delt_Ex = Trasmint_From_Radar_ENU_to_84_1(1,1) * Position_Arrival_ENU(1) + Trasmint_From_Radar_ENU_to_84_1(1,2) * Position_Arrival_ENU(2) + Trasmint_From_Radar_ENU_to_84_1(1,3) * Position_Arrival_ENU(3);
	delt_Ey = Trasmint_From_Radar_ENU_to_84_1(2,1) * Position_Arrival_ENU(1) + Trasmint_From_Radar_ENU_to_84_1(2,2) * Position_Arrival_ENU(2) + Trasmint_From_Radar_ENU_to_84_1(2,3) * Position_Arrival_ENU(3);
	delt_Ez = Trasmint_From_Radar_ENU_to_84_1(3,1) * Position_Arrival_ENU(1) + Trasmint_From_Radar_ENU_to_84_1(3,2) * Position_Arrival_ENU(2) + Trasmint_From_Radar_ENU_to_84_1(3,3) * Position_Arrival_ENU(3);     
	Position_Arrival_WGS(1) = delt_Ex + Exr;        % �õ�WGS84����ϵ��Ŀ��λ������
	Position_Arrival_WGS(2) = delt_Ey + Eyr;
	Position_Arrival_WGS(3) = delt_Ez + Ezr;
    %     Position_Arrival_WGS_store(Times_Filtering,:) = Position_Arrival_WGS;
    
