function Rinv_matrix_Sensor = Rinv_matrix_calc_incomplete_ThetaPhi(Hmatrix_Sensor,Sensor_data)

data_length = length(Sensor_data);

for ii = 1:data_length
    R_matrix_spherical = [(Sensor_data(ii).Theta_Error)^2 0
                           0                              (Sensor_data(ii).Phi_Error)^2];
    Hmatrix = Hmatrix_Sensor(ii).matrix;
    Rinv_matrix = Hmatrix' * inv(R_matrix_spherical) * Hmatrix;
    Rinv_matrix_Sensor(ii,:,:) = Rinv_matrix;
%     R_matrix = inv(Hmatrix) * (R_matrix_spherical) * inv(Hmatrix)';
%     Rinv_matrix_Sensor(ii,:,:) = inv(R_matrix);
end
