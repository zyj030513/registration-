function Plot_figure1(Sensor1_data,Sensor2_data,Sensor3_data,Sensor4_data,Sensor1_data_Filtered,Sensor2_data_Filtered,Sensor3_data_Filtered,Sensor4_data_Filtered,data_length)
for ii = 1:data_length
    Sensor1_x_measured(ii) = Sensor1_data(ii).Tar_Posi(1);
    Sensor1_y_measured(ii) = Sensor1_data(ii).Tar_Posi(2);
    Sensor1_z_measured(ii) = Sensor1_data(ii).Tar_Posi(3);
    
    Sensor2_x_measured(ii) = Sensor2_data(ii).Tar_Posi(1);
    Sensor2_y_measured(ii) = Sensor2_data(ii).Tar_Posi(2);
    Sensor2_z_measured(ii) = Sensor2_data(ii).Tar_Posi(3);
    
    Sensor3_x_measured(ii) = Sensor3_data(ii).Tar_Posi(1);
    Sensor3_y_measured(ii) = Sensor3_data(ii).Tar_Posi(2);
    Sensor3_z_measured(ii) = Sensor3_data(ii).Tar_Posi(3);
    
    Sensor4_x_measured(ii) = Sensor4_data(ii).Tar_Posi(1);
    Sensor4_y_measured(ii) = Sensor4_data(ii).Tar_Posi(2);
    Sensor4_z_measured(ii) = Sensor4_data(ii).Tar_Posi(3);
    
    Sensor1_x_filtered = Sensor1_data_Filtered.X_nn(:,1);
    Sensor1_y_filtered = Sensor1_data_Filtered.X_nn(:,4);
    Sensor1_z_filtered = Sensor1_data_Filtered.X_nn(:,7);
    
    Sensor2_x_filtered = Sensor2_data_Filtered.X_nn(:,1);
    Sensor2_y_filtered = Sensor2_data_Filtered.X_nn(:,4);
    Sensor2_z_filtered = Sensor2_data_Filtered.X_nn(:,7);
    
    Sensor3_x_filtered = Sensor3_data_Filtered.X_nn(:,1);
    Sensor3_y_filtered = Sensor3_data_Filtered.X_nn(:,4);
    Sensor3_z_filtered = Sensor3_data_Filtered.X_nn(:,7);
    
    Sensor4_x_filtered = Sensor4_data_Filtered.X_nn(:,1);
    Sensor4_y_filtered = Sensor4_data_Filtered.X_nn(:,4);
    Sensor4_z_filtered = Sensor4_data_Filtered.X_nn(:,7);
end
figure;
plot(Sensor1_x_measured)
hold on
plot(Sensor1_x_filtered,'r')

figure;
plot(Sensor1_y_measured)
hold on
plot(Sensor1_y_filtered,'r')

figure;
plot(Sensor1_z_measured)
hold on
plot(Sensor1_z_filtered,'r')

figure;
plot(Sensor2_x_measured)
hold on
plot(Sensor2_x_filtered,'r')

figure;
plot(Sensor2_y_measured)
hold on
plot(Sensor2_y_filtered,'r')

figure;
plot(Sensor2_z_measured)
hold on
plot(Sensor2_z_filtered,'r')

figure;
plot(Sensor3_x_measured)
hold on
plot(Sensor3_x_filtered,'r')

figure;
plot(Sensor3_y_measured)
hold on
plot(Sensor3_y_filtered,'r')

figure;
plot(Sensor3_z_measured)
hold on
plot(Sensor3_z_filtered,'r')

figure;
plot(Sensor4_x_measured)
hold on
plot(Sensor4_x_filtered,'r')

figure;
plot(Sensor4_y_measured)
hold on
plot(Sensor4_y_filtered,'r')

figure;
plot(Sensor4_z_measured)
hold on
plot(Sensor4_z_filtered,'r')