function [Position_WGS_Real] = Calc_Real_Position(Fusion_time,Target_ID)

Ts = 0.01;
datalength = length(Fusion_time);

if Target_ID == 1
    % save('.\data\data_track_original\Target_Track_AEW.mat','Trace_output_AEW','Velocity_output_AEW','ShowVr_AEW','Num_AEW','Ts');
    load('.\target-position\data\data_Target_track_wgs84\Trace_AEW_WGS.mat')
    % ��ֵ��ȷ��Ŀ����ʵλ��
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % ǰһ��λ��
        Position_later = ceil(Fusion_time(ii)/Ts);  % ��һ��λ��
        
        if Position_last == Position_later  % �պ���ԭ���ɹ켣�Ĳ������غ�
            Position_WGS_Real(:,ii) = Trace_AEW_WGS(1:3,Position_last);
        else
            Position_WGS_Real(1,ii) = (Trace_AEW_WGS(1,Position_later) - Trace_AEW_WGS(1,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_AEW_WGS(1,Position_last);
            Position_WGS_Real(2,ii) = (Trace_AEW_WGS(2,Position_later) - Trace_AEW_WGS(2,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_AEW_WGS(2,Position_last);
            Position_WGS_Real(3,ii) = (Trace_AEW_WGS(3,Position_later) - Trace_AEW_WGS(3,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_AEW_WGS(3,Position_last);
        end  
    end
elseif Target_ID == 2
    load('D:\ʦ��ʦ������\κ���� - �޸ı仯\С���ķ��� ����������̬����A\Ŀ��ģ�����\data\data_Target_track_wgs84\Trace_EA18G11_WGS.mat')
    % ��ֵ��ȷ��Ŀ����ʵλ��
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % ǰһ��λ��
        Position_later = ceil(Fusion_time(ii)/Ts);  % ��һ��λ��
        
        if Position_last == Position_later  % �պ���ԭ���ɹ켣�Ĳ������غ�
            Position_WGS_Real(:,ii) = Trace_EA18G11_WGS(1:3,Position_last);
        else
            Position_WGS_Real(1,ii) = (Trace_EA18G11_WGS(1,Position_later) - Trace_EA18G11_WGS(1,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G11_WGS(1,Position_last);
            Position_WGS_Real(2,ii) = (Trace_EA18G11_WGS(2,Position_later) - Trace_EA18G11_WGS(2,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G11_WGS(2,Position_last);
            Position_WGS_Real(3,ii) = (Trace_EA18G11_WGS(3,Position_later) - Trace_EA18G11_WGS(3,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G11_WGS(3,Position_last);
        end  
    end
elseif Target_ID == 3
    load('D:\ʦ��ʦ������\κ���� - �޸ı仯\С���ķ��� ����������̬����A\Ŀ��ģ�����\data\data_Target_track_wgs84\Trace_EA18G12_WGS.mat')
    % ��ֵ��ȷ��Ŀ����ʵλ��
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % ǰһ��λ��
        Position_later = ceil(Fusion_time(ii)/Ts);  % ��һ��λ��
        
        if Position_last == Position_later  % �պ���ԭ���ɹ켣�Ĳ������غ�
            Position_WGS_Real(:,ii) = Trace_EA18G12_WGS(1:3,Position_last);
        else
            Position_WGS_Real(1,ii) = (Trace_EA18G12_WGS(1,Position_later) - Trace_EA18G12_WGS(1,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G12_WGS(1,Position_last);
            Position_WGS_Real(2,ii) = (Trace_EA18G12_WGS(2,Position_later) - Trace_EA18G12_WGS(2,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G12_WGS(2,Position_last);
            Position_WGS_Real(3,ii) = (Trace_EA18G12_WGS(3,Position_later) - Trace_EA18G12_WGS(3,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G12_WGS(3,Position_last);
        end  
    end
elseif Target_ID == 4
    load('D:\ʦ��ʦ������\κ���� - �޸ı仯\С���ķ��� ����������̬����A\Ŀ��ģ�����\data\data_Target_track_wgs84\Trace_EA18G21_WGS.mat')
    % ��ֵ��ȷ��Ŀ����ʵλ��
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % ǰһ��λ��
        Position_later = ceil(Fusion_time(ii)/Ts);  % ��һ��λ��
        
        if Position_last == Position_later  % �պ���ԭ���ɹ켣�Ĳ������غ�
            Position_WGS_Real(:,ii) = Trace_EA18G21_WGS(1:3,Position_last);
        else
            Position_WGS_Real(1,ii) = (Trace_EA18G21_WGS(1,Position_later) - Trace_EA18G21_WGS(1,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G21_WGS(1,Position_last);
            Position_WGS_Real(2,ii) = (Trace_EA18G21_WGS(2,Position_later) - Trace_EA18G21_WGS(2,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G21_WGS(2,Position_last);
            Position_WGS_Real(3,ii) = (Trace_EA18G21_WGS(3,Position_later) - Trace_EA18G21_WGS(3,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G21_WGS(3,Position_last);
        end  
    end
elseif Target_ID == 5
    load('D:\ʦ��ʦ������\κ���� - �޸ı仯\С���ķ��� ����������̬����A\Ŀ��ģ�����\data\data_Target_track_wgs84\Trace_EA18G22_WGS.mat')
    % ��ֵ��ȷ��Ŀ����ʵλ��
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % ǰһ��λ��
        Position_later = ceil(Fusion_time(ii)/Ts);  % ��һ��λ��
        
        if Position_last == Position_later  % �պ���ԭ���ɹ켣�Ĳ������غ�
            Position_WGS_Real(:,ii) = Trace_EA18G22_WGS(1:3,Position_last);
        else
            Position_WGS_Real(1,ii) = (Trace_EA18G22_WGS(1,Position_later) - Trace_EA18G22_WGS(1,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G22_WGS(1,Position_last);
            Position_WGS_Real(2,ii) = (Trace_EA18G22_WGS(2,Position_later) - Trace_EA18G22_WGS(2,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G22_WGS(2,Position_last);
            Position_WGS_Real(3,ii) = (Trace_EA18G22_WGS(3,Position_later) - Trace_EA18G22_WGS(3,Position_last)) / Ts * (Fusion_time(ii) - Position_last * Ts) +...
                                       Trace_EA18G22_WGS(3,Position_last);
        end  
    end
else
    disp('�����ڵ�Ŀ�꣬�޷���ȡĿ����ʵλ��')
    
end
    

