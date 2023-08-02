function R_matrix_all = R_matrix_calc(Sensor_data1,Sensor_data2)

R_matrix_spherical1 = [(Sensor_data1.Theta_Error)^2                             0                                           0;
                                        0                           (Sensor_data1.Phi_Error)^2                              0;
                                        0                                       0                               (Sensor_data1.Range_Error)^2];

R_matrix_spherical2 = [(Sensor_data2.Theta_Error)^2                             0                                           0;
                                        0                           (Sensor_data2.Phi_Error)^2                              0;
                                        0                                       0                               (Sensor_data2.Range_Error)^2];
                                    
R_matrix_all = [R_matrix_spherical1 eye(3);
                eye(3)              R_matrix_spherical2];