clc;
close all;
clear all;
format long;
% chouqv_select = [1 2 3 4 5 10] 
% Chouqv_all = 10;
Chouqv_all = 1;
for mm = Chouqv_all:-1:1
    mm
    chouqv = mm;
    Simulation_times = 10;

    beta_19 = [];
    for tt = 1:Simulation_times       
        dir_name = strcat('Registration_19_chouqv_',num2str(chouqv));
        data_name = ['.\',dir_name,'\Data_Registrated_' int2str(tt) '.mat'];
        load(data_name);
        beta_19(tt,:) = beta;
    end
    %%
    
    [Radar_Number,RadarPar] = Radar_Prameters_Set_J();
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
    
    % 计算偏差估计结果
    for tt = 1:Simulation_times
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
    end
    
    RMSE_beta_19_S1_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_theta))/pi*180    
    RMSE_beta_19_S1_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_phi))/pi*180
    RMSE_beta_19_S1_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S1_range))
    
    RMSE_beta_19_S2_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_theta))/pi*180
    RMSE_beta_19_S2_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_phi))/pi*180
    RMSE_beta_19_S2_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S2_range))
    
    RMSE_beta_19_S3_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_theta))/pi*180
    RMSE_beta_19_S3_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_phi))/pi*180
    RMSE_beta_19_S3_range(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S3_range))
    
    RMSE_beta_19_S4_theta(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S4_theta))/pi*180
    RMSE_beta_19_S4_phi(Chouqv_all + 1 - mm) = sqrt(mse(Error_beta_19_S4_phi))/pi*180
    
end

figure
plot(RMSE_beta_19_S1_theta,'k')
title('S1 theta')

figure
plot(RMSE_beta_19_S1_phi,'k')
title('S1 phi')


figure
plot(RMSE_beta_19_S1_range,'k')
title('S1 range')


figure
plot(RMSE_beta_19_S2_theta,'k')
title('S2 theta')

figure
plot(RMSE_beta_19_S2_phi,'k')
title('S2 phi')

figure
plot(RMSE_beta_19_S2_range,'k')
title('S2 range')


figure
plot(RMSE_beta_19_S3_theta,'k')
title('S3 theta')

figure
plot(RMSE_beta_19_S3_phi,'k')
title('S3 phi')

figure
plot(RMSE_beta_19_S3_range,'k')
title('S3 range')


figure
plot(RMSE_beta_19_S4_theta,'k')
title('S4 theta')

figure
plot(RMSE_beta_19_S4_phi,'k')
title('S4 phi')

