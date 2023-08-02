clc;
clear all;
close all;

Simulation_times = 100;
% % 传感器数目为4
mkdir('C:\Users\ww\Desktop\小论文仿真 （不包含姿态角误差）\数据预处理\data')
for tt = 1:Simulation_times
    tt
    file_name = strcat('C:\Users\ww\Desktop\小论文仿真 （不包含姿态角误差）\目标模拟程序\data\no_Interupt\Data_Output_H_J_no_Interupt_',int2str(tt),'.mat');
    load(file_name);
    Sensor1_data = Target_Trace_WGS_Meas(1:4:end);
    Sensor2_data = Target_Trace_WGS_Meas(2:4:end);
    Sensor3_data = Target_Trace_WGS_Meas(3:4:end);
    Sensor4_data = Target_Trace_WGS_Meas(4:4:end);
    
    data_length = Target_Trace_WGS_Meas_radar_Len / 4;
    
    data_name = ['.\data\Data_for_4_Sensors_' int2str(tt) '.mat'];
    save(data_name,'Sensor1_data','Sensor2_data','Sensor3_data','Sensor4_data','data_length');

end