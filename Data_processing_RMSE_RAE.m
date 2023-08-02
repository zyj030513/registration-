clc;
close all;
clear all;
format long;
% chouqv_select = [1 2 3 4 5 10] 
Chouqv_all = 10;
for mm = Chouqv_all:-1:1
    mm
    chouqv = mm;
    Simulation_times = 100;
    beta_9 = [];
    beta_15 = [];
    beta_19 = [];
    beta_20 = [];
    for tt = 1:Simulation_times
        dir_name = strcat('Registration_9_chouqv_',num2str(chouqv));
        data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
        load(data_name);
        beta_9(tt,:) = beta;
        
        dir_name = strcat('Registration_15_chouqv_',num2str(chouqv));
        data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
        load(data_name);
        beta_15(tt,:) = beta;
        
        
        dir_name = strcat('Registration_19_chouqv_',num2str(chouqv));
        data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
        load(data_name);
        beta_19(tt,:) = beta;
        
        dir_name = strcat('Registration_20_chouqv_',num2str(chouqv));
        data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
        load(data_name);
        beta_20(tt,:) = beta;
    end
    %%
    
    [Radar_Number,RadarPar] = Radar_Prameters_Set_J();
    Error_beta_9_S1_theta = [];
    Error_beta_9_S1_phi = [];
    Error_beta_9_S1_range = [];
    Error_beta_9_S2_theta = [];
    Error_beta_9_S2_phi = [];
    Error_beta_9_S2_range = [];
    Error_beta_9_S3_theta = [];
    Error_beta_9_S3_phi = [];
    Error_beta_9_S3_range = [];
    Error_beta_9_S4_theta = [];
    Error_beta_9_S4_phi = [];
    
    Error_beta_15_S1_theta = [];
    Error_beta_15_S1_phi = [];
    Error_beta_15_S1_range = [];
    Error_beta_15_S2_theta = [];
    Error_beta_15_S2_phi = [];
    Error_beta_15_S2_range = [];
    Error_beta_15_S3_theta = [];
    Error_beta_15_S3_phi = [];
    Error_beta_15_S3_range = [];
    Error_beta_15_S4_theta = [];
    Error_beta_15_S4_phi = [];
    
    Error_beta_19_S1_theta = [];
    Error_beta_19_S1_phi = [];
    Error_beta_19_S1_range = [];
    Error_beta_19_S2_theta = [];
    Error_beta_19_S2_phi = [];
    Error_beta_19_S2_range = [];
    Error_beta_19_S3_theta = [];
    Error_beta_19_S3_phi = [];
    Error_beta_19_S3_range = [];
    Error_beta_19_S4_theta = [];
    Error_beta_19_S4_phi = [];
    
    Error_beta_20_S1_theta = [];
    Error_beta_20_S1_phi = [];
    Error_beta_20_S1_range = [];
    Error_beta_20_S2_theta = [];
    Error_beta_20_S2_phi = [];
    Error_beta_20_S2_range = [];
    Error_beta_20_S3_theta = [];
    Error_beta_20_S3_phi = [];
    Error_beta_20_S3_range = [];
    Error_beta_20_S4_theta = [];
    Error_beta_20_S4_phi = [];
    % 计算偏差估计结果
    for tt = 1:Simulation_times
        % 9
        Error_beta_9_S1_theta(tt) = beta_9(tt,1) - RadarPar(1,1).theta_bias;
        Error_beta_9_S1_phi(tt) = beta_9(tt,2) - RadarPar(1,1).phi_bias;
        Error_beta_9_S1_range(tt) = beta_9(tt,3) - RadarPar(1,1).range_bias;
        Error_beta_9_S2_theta(tt) = beta_9(tt,4) - RadarPar(1,2).theta_bias;
        Error_beta_9_S2_phi(tt) = beta_9(tt,5) - RadarPar(1,2).phi_bias;
        Error_beta_9_S2_range(tt) = beta_9(tt,6) - RadarPar(1,2).range_bias;
        Error_beta_9_S3_theta(tt) = beta_9(tt,7) - RadarPar(1,3).theta_bias;
        Error_beta_9_S3_phi(tt) = beta_9(tt,8) - RadarPar(1,3).phi_bias;
        Error_beta_9_S3_range(tt) = beta_9(tt,9) - RadarPar(1,3).range_bias;
        Error_beta_9_S4_theta(tt) = beta_9(tt,10) - RadarPar(1,4).theta_bias;
        Error_beta_9_S4_phi(tt) = beta_9(tt,11) - RadarPar(1,4).phi_bias;
        % 15
        Error_beta_15_S1_theta(tt) = beta_15(tt,1) - RadarPar(1,1).theta_bias;
        Error_beta_15_S1_phi(tt) = beta_15(tt,2) - RadarPar(1,1).phi_bias;
        Error_beta_15_S1_range(tt) = beta_15(tt,3) - RadarPar(1,1).range_bias;
        Error_beta_15_S2_theta(tt) = beta_15(tt,4) - RadarPar(1,2).theta_bias;
        Error_beta_15_S2_phi(tt) = beta_15(tt,5) - RadarPar(1,2).phi_bias;
        Error_beta_15_S2_range(tt) = beta_15(tt,6) - RadarPar(1,2).range_bias;
        Error_beta_15_S3_theta(tt) = beta_15(tt,7) - RadarPar(1,3).theta_bias;
        Error_beta_15_S3_phi(tt) = beta_15(tt,8) - RadarPar(1,3).phi_bias;
        Error_beta_15_S3_range(tt) = beta_15(tt,9) - RadarPar(1,3).range_bias;
        Error_beta_15_S4_theta(tt) = beta_15(tt,10) - RadarPar(1,4).theta_bias;
        Error_beta_15_S4_phi(tt) = beta_15(tt,11) - RadarPar(1,4).phi_bias;
        % 19
        Error_beta_19_S1_theta(tt) = beta_19(tt,1) - RadarPar(1,1).theta_bias;
        Error_beta_19_S1_phi(tt) = beta_19(tt,2) - RadarPar(1,1).phi_bias;
        Error_beta_19_S1_range(tt) = beta_19(tt,3) - RadarPar(1,1).range_bias;
        Error_beta_19_S2_theta(tt) = beta_19(tt,4) - RadarPar(1,2).theta_bias;
        Error_beta_19_S2_phi(tt) = beta_19(tt,5) - RadarPar(1,2).phi_bias;
        Error_beta_19_S2_range(tt) = beta_19(tt,6) - RadarPar(1,2).range_bias;
        Error_beta_19_S3_theta(tt) = beta_19(tt,7) - RadarPar(1,3).theta_bias;
        Error_beta_19_S3_phi(tt) = beta_19(tt,8) - RadarPar(1,3).phi_bias;
        Error_beta_19_S3_range(tt) = beta_19(tt,9) - RadarPar(1,3).range_bias;
        Error_beta_19_S4_theta(tt) = beta_19(tt,10) - RadarPar(1,4).theta_bias;
        Error_beta_19_S4_phi(tt) = beta_19(tt,11) - RadarPar(1,4).phi_bias;
        % 20
        Error_beta_20_S1_theta(tt) = beta_20(tt,1) - RadarPar(1,1).theta_bias;
        Error_beta_20_S1_phi(tt) = beta_20(tt,2) - RadarPar(1,1).phi_bias;
        Error_beta_20_S1_range(tt) = beta_20(tt,3) - RadarPar(1,1).range_bias;
        Error_beta_20_S2_theta(tt) = beta_20(tt,4) - RadarPar(1,2).theta_bias;
        Error_beta_20_S2_phi(tt) = beta_20(tt,5) - RadarPar(1,2).phi_bias;
        Error_beta_20_S2_range(tt) = beta_20(tt,6) - RadarPar(1,2).range_bias;
        Error_beta_20_S3_theta(tt) = beta_20(tt,7) - RadarPar(1,3).theta_bias;
        Error_beta_20_S3_phi(tt) = beta_20(tt,8) - RadarPar(1,3).phi_bias;
        Error_beta_20_S3_range(tt) = beta_20(tt,9) - RadarPar(1,3).range_bias;
        Error_beta_20_S4_theta(tt) = beta_20(tt,10) - RadarPar(1,4).theta_bias;
        Error_beta_20_S4_phi(tt) = beta_20(tt,11) - RadarPar(1,4).phi_bias;
    end
    RMSE_beta_9_S1_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S1_theta))/pi*180
    RMSE_beta_15_S1_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S1_theta))/pi*180
    RMSE_beta_19_S1_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_theta))/pi*180
    RMSE_beta_20_S1_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S1_theta))/pi*180
    RMSE_beta_9_S1_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S1_phi))/pi*180
    RMSE_beta_15_S1_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S1_phi))/pi*180
    RMSE_beta_19_S1_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_phi))/pi*180
    RMSE_beta_20_S1_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S1_phi))/pi*180
    RMSE_beta_9_S1_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S1_range))
    RMSE_beta_15_S1_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S1_range))
    RMSE_beta_19_S1_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_range))
    RMSE_beta_20_S1_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S1_range))
    
    RMSE_beta_9_S2_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S2_theta))/pi*180
    RMSE_beta_15_S2_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S2_theta))/pi*180
    RMSE_beta_19_S2_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_theta))/pi*180
    RMSE_beta_20_S2_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S2_theta))/pi*180
    RMSE_beta_9_S2_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S2_phi))/pi*180
    RMSE_beta_15_S2_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S2_phi))/pi*180
    RMSE_beta_19_S2_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_phi))/pi*180
    RMSE_beta_20_S2_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S2_phi))/pi*180
    RMSE_beta_9_S2_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S2_range))
    RMSE_beta_15_S2_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S2_range))
    RMSE_beta_19_S2_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_range))
    RMSE_beta_20_S2_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S2_range))
    
    RMSE_beta_9_S3_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S3_theta))/pi*180
    RMSE_beta_15_S3_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S3_theta))/pi*180
    RMSE_beta_19_S3_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_theta))/pi*180
    RMSE_beta_20_S3_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S3_theta))/pi*180
    RMSE_beta_9_S3_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S3_phi))/pi*180
    RMSE_beta_15_S3_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S3_phi))/pi*180
    RMSE_beta_19_S3_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_phi))/pi*180
    RMSE_beta_20_S3_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S3_phi))/pi*180
    RMSE_beta_9_S3_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S3_range))
    RMSE_beta_15_S3_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S3_range))
    RMSE_beta_19_S3_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_range))
    RMSE_beta_20_S3_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S3_range))
    
    RMSE_beta_9_S4_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S4_theta))/pi*180
    RMSE_beta_15_S4_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S4_theta))/pi*180
    RMSE_beta_19_S4_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S4_theta))/pi*180
    RMSE_beta_20_S4_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S4_theta))/pi*180
    RMSE_beta_9_S4_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_9_S4_phi))/pi*180
    RMSE_beta_15_S4_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_15_S4_phi))/pi*180
    RMSE_beta_19_S4_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S4_phi))/pi*180
    RMSE_beta_20_S4_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_20_S4_phi))/pi*180
    
end

figure
plot(RMSE_beta_9_S1_theta)
hold on
plot(RMSE_beta_15_S1_theta,'g')
hold on
plot(RMSE_beta_19_S1_theta,'k')
hold on
plot(RMSE_beta_20_S1_theta,'r')
title('S1 theta')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S1_phi)
hold on
plot(RMSE_beta_15_S1_phi,'g')
hold on
plot(RMSE_beta_19_S1_phi,'k')
hold on
plot(RMSE_beta_20_S1_phi,'r')
title('S1 phi')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S1_range)
hold on
plot(RMSE_beta_15_S1_range,'g')
hold on
plot(RMSE_beta_19_S1_range,'k')
hold on
plot(RMSE_beta_20_S1_range,'r')
title('S1 range')
legend('A9','A15','A19','A20')

figure
plot(RMSE_beta_9_S2_theta)
hold on
plot(RMSE_beta_15_S2_theta,'g')
hold on
plot(RMSE_beta_19_S2_theta,'k')
hold on
plot(RMSE_beta_20_S2_theta,'r')
title('S2 theta')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S2_phi)
hold on
plot(RMSE_beta_15_S2_phi,'g')
hold on
plot(RMSE_beta_19_S2_phi,'k')
hold on
plot(RMSE_beta_20_S2_phi,'r')
title('S2 phi')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S2_range)
hold on
plot(RMSE_beta_15_S2_range,'g')
hold on
plot(RMSE_beta_19_S2_range,'k')
hold on
plot(RMSE_beta_20_S2_range,'r')
title('S2 range')
legend('A9','A15','A19','A20')

figure
plot(RMSE_beta_9_S3_theta)
hold on
plot(RMSE_beta_15_S3_theta,'g')
hold on
plot(RMSE_beta_19_S3_theta,'k')
hold on
plot(RMSE_beta_20_S3_theta,'r')
title('S3 theta')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S3_phi)
hold on
plot(RMSE_beta_15_S3_phi,'g')
hold on
plot(RMSE_beta_19_S3_phi,'k')
hold on
plot(RMSE_beta_20_S3_phi,'r')
title('S3 phi')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S3_range)
hold on
plot(RMSE_beta_15_S3_range,'g')
hold on
plot(RMSE_beta_19_S3_range,'k')
hold on
plot(RMSE_beta_20_S3_range,'r')
title('S3 range')
legend('A9','A15','A19','A20')

figure
plot(RMSE_beta_9_S4_theta)
hold on
plot(RMSE_beta_15_S4_theta,'g')
hold on
plot(RMSE_beta_19_S4_theta,'k')
hold on
plot(RMSE_beta_20_S4_theta,'r')
title('S4 theta')
legend('A9','A15','A19','A20')
figure
plot(RMSE_beta_9_S4_phi)
hold on
plot(RMSE_beta_15_S4_phi,'g')
hold on
plot(RMSE_beta_19_S4_phi,'k')
hold on
plot(RMSE_beta_20_S4_phi,'r')
title('S4 phi')
legend('A9','A15','A19','A20')
