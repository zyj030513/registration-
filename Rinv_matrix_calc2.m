function Rinv_matrix_Sensor = Rinv_matrix_calc2(Sensor_data)

data_length = length(Sensor_data);

for ii = 1:data_length
    L_Sensor = Sensor_data(ii).Sensor_L;
    B_Sensor = Sensor_data(ii).Sensor_B;
    H_Sensor = Sensor_data(ii).Sensor_H;
    
    % 84到东北天的旋转矩阵
    H1_b = [-sin(L_Sensor)    -cos(L_Sensor) * sin(B_Sensor)    cos(L_Sensor) * cos(B_Sensor);...
            cos(L_Sensor)     -sin(L_Sensor) * sin(B_Sensor)    sin(L_Sensor) * cos(B_Sensor);...
            0                 cos(B_Sensor)                     sin(B_Sensor)].';
    Tar_Theta = Sensor_data(ii).Tar_Theta;
    Tar_Phi = Sensor_data(ii).Tar_Phi;
    Tar_R = Sensor_data(ii).Tar_R;    
    
       
    Jn1 = [-Tar_R * cos(Tar_Phi) * sin(Tar_Theta)   -Tar_R * sin(Tar_Phi) * cos(Tar_Theta)      cos(Tar_Phi) * cos(Tar_Theta);
           Tar_R * cos(Tar_Phi) * cos(Tar_Theta)    -Tar_R * sin(Tar_Phi) * sin(Tar_Theta)      cos(Tar_Phi) * sin(Tar_Theta);
           0                                        Tar_R * cos(Tar_Phi)                        sin(Tar_Phi)];   
    
    Hmatrix = H1_b.' * Jn1;
    R_matrix_spherical = [(Sensor_data(ii).Theta_Error)^2   0                               0;
                                0                           (Sensor_data(ii).Phi_Error)^2         0;
                                0                           0                               (Sensor_data(ii).Range_Error)^2];
        
    
    R_matrix = Hmatrix * (R_matrix_spherical) * Hmatrix';
    Rinv_matrix_Sensor(ii,:,:) = pinv(R_matrix);
%     R_matrix = inv(Hmatrix) * (R_matrix_spherical) * inv(Hmatrix)';
%     Rinv_matrix_Sensor(ii,:,:) = inv(R_matrix);
end
