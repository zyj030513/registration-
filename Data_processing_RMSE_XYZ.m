clc;
close all;
clear all;
format long;

Simulation_times = 100;

chouqv = 1;
for tt = 1:Simulation_times
    tt
    %     data_name = ['.\Registration_19\Data_Registrated_' int2str(tt) '.mat'];
    dir_name = strcat('Registration_19_chouqv_',num2str(chouqv));
    data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
    
    load(data_name);
    %     Sensor_fusion_19 = data_Fusion;
    %         beta_19(tt,:) = beta;
    Error_x_19(tt,:) = Error_Fusion_x;
    Error_y_19(tt,:) = Error_Fusion_y;
    Error_z_19(tt,:) = Error_Fusion_z;
    %     bias_attitude_19(tt,:) = bias_attitude;
    
    %     data_name = ['.\Registration_20\Data_Registrated_' int2str(tt) '.mat'];
    dir_name = strcat('Registration_20_chouqv_',num2str(chouqv));
    data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
    load(data_name);
    %     Sensor_fusion_20 = data_Fusion;
    %         beta_20(tt,:) = beta;
    Error_x_20(tt,:) = Error_Fusion_x;
    Error_y_20(tt,:) = Error_Fusion_y;
    Error_z_20(tt,:) = Error_Fusion_z;
    %     bias_attitude_20(tt,:) = bias_attitude;
end
%%
data_length = length(Error_Fusion_x);
for nn = 1:data_length
    RMSE_X_FUSION_19(nn) = sqrt(sum((Error_x_19(:,nn).^2))/Simulation_times);
    RMSE_Y_FUSION_19(nn) = sqrt(sum((Error_y_19(:,nn).^2))/Simulation_times);
    RMSE_Z_FUSION_19(nn) = sqrt(sum((Error_z_19(:,nn).^2))/Simulation_times);
    RMSE_ALL_FUSION_19(nn) = sqrt(RMSE_X_FUSION_19(nn)^2 + RMSE_Y_FUSION_19(nn)^2 + RMSE_Z_FUSION_19(nn)^2);
    
    RMSE_X_FUSION_20(nn) = sqrt(sum((Error_x_20(:,nn).^2))/Simulation_times);
    RMSE_Y_FUSION_20(nn) = sqrt(sum((Error_y_20(:,nn).^2))/Simulation_times);
    RMSE_Z_FUSION_20(nn) = sqrt(sum((Error_z_20(:,nn).^2))/Simulation_times);
    RMSE_ALL_FUSION_20(nn) = sqrt(RMSE_X_FUSION_20(nn)^2 + RMSE_Y_FUSION_20(nn)^2 + RMSE_Z_FUSION_20(nn)^2);
end
figure
plot(RMSE_X_FUSION_19)
hold on
plot(RMSE_X_FUSION_20,'r')
title('Rmse of x')

figure
plot(RMSE_Y_FUSION_19)
hold on
plot(RMSE_Y_FUSION_20,'r')
title('Rmse of y')

figure
plot(RMSE_Z_FUSION_19)
hold on
plot(RMSE_Z_FUSION_20,'r')
title('Rmse of z')

figure
plot(RMSE_ALL_FUSION_19)
hold on
plot(RMSE_ALL_FUSION_20,'r')
title('Rmse of all')