function [Position_WGS_Real] = Calc_Real_Position(Fusion_time,Target_ID)

Ts = 0.01;
datalength = length(Fusion_time);

if Target_ID == 1
    % save('.\data\data_track_original\Target_Track_AEW.mat','Trace_output_AEW','Velocity_output_AEW','ShowVr_AEW','Num_AEW','Ts');
    load('.\target-position\data\data_Target_track_wgs84\Trace_AEW_WGS.mat')
    % 插值法确定目标真实位置
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % 前一点位置
        Position_later = ceil(Fusion_time(ii)/Ts);  % 后一点位置
        
        if Position_last == Position_later  % 刚好与原生成轨迹的采样点重合
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
    load('D:\师兄师姐资料\魏子翔 - 修改变化\小论文仿真 （不包含姿态角误差）A\目标模拟程序\data\data_Target_track_wgs84\Trace_EA18G11_WGS.mat')
    % 插值法确定目标真实位置
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % 前一点位置
        Position_later = ceil(Fusion_time(ii)/Ts);  % 后一点位置
        
        if Position_last == Position_later  % 刚好与原生成轨迹的采样点重合
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
    load('D:\师兄师姐资料\魏子翔 - 修改变化\小论文仿真 （不包含姿态角误差）A\目标模拟程序\data\data_Target_track_wgs84\Trace_EA18G12_WGS.mat')
    % 插值法确定目标真实位置
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % 前一点位置
        Position_later = ceil(Fusion_time(ii)/Ts);  % 后一点位置
        
        if Position_last == Position_later  % 刚好与原生成轨迹的采样点重合
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
    load('D:\师兄师姐资料\魏子翔 - 修改变化\小论文仿真 （不包含姿态角误差）A\目标模拟程序\data\data_Target_track_wgs84\Trace_EA18G21_WGS.mat')
    % 插值法确定目标真实位置
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % 前一点位置
        Position_later = ceil(Fusion_time(ii)/Ts);  % 后一点位置
        
        if Position_last == Position_later  % 刚好与原生成轨迹的采样点重合
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
    load('D:\师兄师姐资料\魏子翔 - 修改变化\小论文仿真 （不包含姿态角误差）A\目标模拟程序\data\data_Target_track_wgs84\Trace_EA18G22_WGS.mat')
    % 插值法确定目标真实位置
    for ii = 1:datalength
        Position_last = floor(Fusion_time(ii)/Ts);  % 前一点位置
        Position_later = ceil(Fusion_time(ii)/Ts);  % 后一点位置
        
        if Position_last == Position_later  % 刚好与原生成轨迹的采样点重合
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
    disp('不存在的目标，无法提取目标真实位置')
    
end
    

