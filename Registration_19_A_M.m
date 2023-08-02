% ������㷨����Registration_15�㷨�����ϣ���԰������ַ��걸�������׼��������о�����������˲���EKF�㷨�����ں�ֵ�󣬲���Registration_15�㷨��ƫ�������׼��
% ������4����ֻ������λ�ǣ�������б��͸����ǡ�
% ��Registration_19_A�������޸���ͼXY,XZ,YZ
clc;
close all;
clear all;
format long;
Simulation_times = 10;  % �������
chouqv = 1;  % ��ȡ����
for nn = 1:Simulation_times
    nn
    file_name = ['.\measurement\Data_for_4_Sensors_' int2str(nn) '.mat'];
    load(file_name);
    %  ��ԭʼ���ݽ��г�ȡ
    
    data_length_all = length(Sensor1_data);
    data_length = 0;
    for ii = 1:chouqv:data_length_all
        data_length = data_length + 1;
        Sensor1_data_chouqv(data_length) = Sensor1_data(ii);
        Sensor2_data_chouqv(data_length) = Sensor2_data(ii);
        Sensor3_data_chouqv(data_length) = Sensor3_data(ii);
        Sensor4_data_chouqv(data_length) = Sensor4_data(ii); 
        Fusion_time(1,data_length) = Sensor1_data_chouqv(data_length).Time_Meas;
    end
    
    beta1 = zeros(11,1);
    beta = zeros(11,1);
    bias_attitude = zeros(12,1);

    for tt = 1:10  % ��������
%         tt
        Sensor1_data_registrated = recalc_Posi(Sensor1_data_chouqv,beta(1:3),bias_attitude(1:3));
        Sensor2_data_registrated = recalc_Posi(Sensor2_data_chouqv,beta(4:6),bias_attitude(4:6));
        Sensor3_data_registrated = recalc_Posi(Sensor3_data_chouqv,beta(7:9),bias_attitude(7:9));
        Sensor4_data_registrated = recalc_Posi(Sensor4_data_chouqv,[beta(10),beta(11),0],bias_attitude(10:12));
        
        Hmatrix_Sensor1_with_measdata = Hmatrix_calc_With_Measured_data(Sensor1_data_registrated);
        Hmatrix_Sensor2_with_measdata = Hmatrix_calc_With_Measured_data(Sensor2_data_registrated);
        Hmatrix_Sensor3_with_measdata = Hmatrix_calc_With_Measured_data(Sensor3_data_registrated);
        Hmatrix_Sensor4_with_measdata_all = Hmatrix_calc_With_Measured_data(Sensor4_data_registrated);
        for ii = 1:data_length
            Hmatrix_Sensor4_with_measdata(ii).matrix = Hmatrix_Sensor4_with_measdata_all(ii).matrix(1:2,:);
        end
        
        Rinv_matrix_Sensor1 = Rinv_matrix_calc(Hmatrix_Sensor1_with_measdata,Sensor1_data_registrated);
        Rinv_matrix_Sensor2 = Rinv_matrix_calc(Hmatrix_Sensor2_with_measdata,Sensor2_data_registrated);
        Rinv_matrix_Sensor3 = Rinv_matrix_calc(Hmatrix_Sensor3_with_measdata,Sensor3_data_registrated);
        %     Rinv_matrix_Sensor4 = Rinv_matrix_calc_incomplete_Theta(Hmatrix_Sensor4_with_measdata,Sensor4_data_registrated);
        Rinv_matrix_Sensor4 = Rinv_matrix_calc_incomplete_ThetaPhi(Hmatrix_Sensor4_with_measdata,Sensor4_data_registrated);
        
        for ii = 1:data_length
            Rinv_matrix(1).matrix = squeeze(Rinv_matrix_Sensor1(ii,:,:));
            Rinv_matrix(2).matrix = squeeze(Rinv_matrix_Sensor2(ii,:,:));
            Rinv_matrix(3).matrix = squeeze(Rinv_matrix_Sensor3(ii,:,:));
            Rinv_matrix(4).matrix = squeeze(Rinv_matrix_Sensor4(ii,:,:));
            
            R_fusion_matrixtmp1 = inv(Rinv_matrix(1).matrix + Rinv_matrix(2).matrix + Rinv_matrix(3).matrix);
            R_fusion_matrixtmp2 = inv(Rinv_matrix(1).matrix + Rinv_matrix(2).matrix + Rinv_matrix(3).matrix + Rinv_matrix(4).matrix);
            R_fusion_matrix1(ii,:,:) = R_fusion_matrixtmp1;
            R_fusion_matrix2(ii,:,:) = R_fusion_matrixtmp2;
            data_Fusion_1 = R_fusion_matrixtmp1 * ...
                (Rinv_matrix(1).matrix * Sensor1_data_registrated(ii).Tar_Posi +...
                Rinv_matrix(2).matrix * Sensor2_data_registrated(ii).Tar_Posi +...
                Rinv_matrix(3).matrix * Sensor3_data_registrated(ii).Tar_Posi);
            % ����EKF ������˲������Է��걸��������ں�
            R_matrix_spherical = [(Sensor4_data_registrated(ii).Theta_Error)^2  0
                0                                           (Sensor4_data_registrated(ii).Phi_Error)^2];
            K_matrix(ii).matrix = R_fusion_matrixtmp2 * Hmatrix_Sensor4_with_measdata(ii).matrix.' * inv(R_matrix_spherical);
            %         K_matrix(ii).matrix = R_fusion_matrixtmp2 * Hmatrix_Sensor4_with_measdata(ii).matrix' * Rinv_matrix(4).matrix;
            Sensor4_data_fusion1 = recalc_Posi_Celiang_Fusion(Sensor4_data_registrated(ii),data_Fusion_1.');
            Z4 = [Sensor4_data_registrated(ii).Tar_Theta; Sensor4_data_registrated(ii).Tar_Phi];
            deltaX = Z4 - Sensor4_data_fusion1(1:2).';
            if deltaX(1) >= pi
                deltaX(1) = deltaX(1) - 2*pi;
            elseif deltaX(1) < -pi
                deltaX(1) = deltaX(1) + 2*pi;
            end
            if deltaX(2) >= pi
                deltaX(2) = deltaX(2) - 2*pi;
            elseif deltaX(2) < -pi
                deltaX(2) = deltaX(2) + 2*pi;
            end
            data_Fusion_2 = data_Fusion_1 + K_matrix(ii).matrix * (deltaX); % Sensor4_data_fusion1λ��1��theta��
            data_Fusion(ii,:) = data_Fusion_2;
            
            R_matrix_spherical_Sensor1 = [(Sensor1_data_registrated(ii).Theta_Error)^2   0                                                 0;
                0                                            (Sensor1_data_registrated(ii).Phi_Error)^2         0;
                0                                             0                                                (Sensor1_data_registrated(ii).Range_Error)^2];
            R_matrix_spherical_Sensor2 = [(Sensor2_data_registrated(ii).Theta_Error)^2   0                                                 0;
                0                                            (Sensor2_data_registrated(ii).Phi_Error)^2         0;
                0                                             0                                                (Sensor2_data_registrated(ii).Range_Error)^2];
            R_matrix_spherical_Sensor3 = [(Sensor3_data_registrated(ii).Theta_Error)^2   0                                                 0;
                0                                            (Sensor3_data_registrated(ii).Phi_Error)^2         0;
                0                                             0                                                (Sensor3_data_registrated(ii).Range_Error)^2];
            R_matrix_spherical_Sensor4 = [(Sensor4_data_registrated(ii).Theta_Error)^2   0
                0                         (Sensor4_data_registrated(ii).Phi_Error)^2];
            
            Sigma_inv = blkdiag(inv(R_matrix_spherical_Sensor1),inv(R_matrix_spherical_Sensor2),inv(R_matrix_spherical_Sensor3),inv(R_matrix_spherical_Sensor4));
            Sigma_inv_all(ii).matrix = Sigma_inv;
            
            
            % ��¼��׼ǰ���ںϽ��
            if tt == 1
                data_Fusion_befor_reg(ii,:) = data_Fusion_2;
            end
        end
        %     [Sensor_fusion_Filtered] = Kalman_filter2(data_Fusion,R_fusion_matrix,data_length);
        
        
        
        Hmatrix_Sensor1 = Hmatrix_calc_With_fusion_data_inv(data_Fusion,Sensor1_data_registrated);
        Hmatrix_Sensor2 = Hmatrix_calc_With_fusion_data_inv(data_Fusion,Sensor2_data_registrated);
        Hmatrix_Sensor3 = Hmatrix_calc_With_fusion_data_inv(data_Fusion,Sensor3_data_registrated);
        Hmatrix_Sensor4_all = Hmatrix_calc_With_fusion_data_inv(data_Fusion,Sensor4_data_registrated);
        for ii = 1:data_length
            Hmatrix_Sensor4(ii).matrix = Hmatrix_Sensor4_all(ii).matrix(:,1:2);
        end
        
        Hmatrix_Sensor_all = [Hmatrix_Sensor1;Hmatrix_Sensor2;Hmatrix_Sensor3;Hmatrix_Sensor4];
        
        [HA_x_matrix_Sensor1,HA_pry_matrix_Sensor1] = HAxpry_matrix_calc_With_Measured_data(data_Fusion,Sensor1_data_registrated);
        [HA_x_matrix_Sensor2,HA_pry_matrix_Sensor2] = HAxpry_matrix_calc_With_Measured_data(data_Fusion,Sensor2_data_registrated);
        [HA_x_matrix_Sensor3,HA_pry_matrix_Sensor3] = HAxpry_matrix_calc_With_Measured_data(data_Fusion,Sensor3_data_registrated);
        [HA_x_matrix_Sensor4,HA_pry_matrix_Sensor4] = HAxpry_matrix_calc_With_Measured_data(data_Fusion,Sensor4_data_registrated);
        
        
        % ����delta x �� delta beta
        for ii = 1:data_length
            Rinv_matrix(1).matrix = squeeze(Rinv_matrix_Sensor1(ii,:,:));
            Rinv_matrix(2).matrix = squeeze(Rinv_matrix_Sensor2(ii,:,:));
            Rinv_matrix(3).matrix = squeeze(Rinv_matrix_Sensor3(ii,:,:));
            %         Rinv_matrix(4).matrix = squeeze(Rinv_matrix_Sensor4(ii,:,:));
            R_fusion_matrixtmp1 = squeeze(R_fusion_matrix1(ii,:,:));
            %         R_fusion_matrix2 = squeeze(R_fusion_matrix2(ii,:,:));
            for aa = 1:3
                Q_matrix_temp(1:3,aa*3-2:aa*3) = R_fusion_matrixtmp1 * Rinv_matrix(aa).matrix * Hmatrix_Sensor_all(aa,ii).matrix;
            end
            Q_matrix_temp(1:3,10:11) = 0;
            
            % ��ǰ�����걸�������ںϽ��ת����������4��������ϵ�£�����ƫ��,ǰ�벿����λ������벿����ת����������4����ϵ�µ���̬��ƫ����������
            Delta_HX_4 = HA_x_matrix_Sensor4(ii).matrix(1:2,:) * Q_matrix_temp;
            % ������4�Ĳ���ƫ��
            Delta_Z = [zeros(2,3*3), eye(2)];
            % �������EKF�ںϺ��ƫ��   data_Fusion_1 + K_matrix(ii).matrix * (Sensor4_data_registrated(ii).Tar_Theta - Sensor4_data_fusion1(1));
            Delta_fusion(ii).matrix = Q_matrix_temp + K_matrix(ii).matrix * (Delta_Z - Delta_HX_4);
            % ��84����ϵ���ܵ�λ�����ת��������������������ϵ
            Delta_x_Sensor1(ii).matrix = HA_x_matrix_Sensor1(ii).matrix * Delta_fusion(ii).matrix;
            Delta_x_Sensor2(ii).matrix = HA_x_matrix_Sensor2(ii).matrix * Delta_fusion(ii).matrix;
            Delta_x_Sensor3(ii).matrix = HA_x_matrix_Sensor3(ii).matrix * Delta_fusion(ii).matrix;
            Delta_x_Sensor4(ii).matrix = HA_x_matrix_Sensor4(ii).matrix(1:2,:) * Delta_fusion(ii).matrix;
            Delta_x_all = -[Delta_x_Sensor1(ii).matrix;Delta_x_Sensor2(ii).matrix;Delta_x_Sensor3(ii).matrix;Delta_x_Sensor4(ii).matrix];
            % ǰ�벿��������ֵ����벿���ǽ�84����ϵ���ܵ�λ�����ת��������������������ϵ������̬�������������
            Delta_beta_Sensor1 = eye(3);
            Delta_beta_Sensor2 = eye(3);
            Delta_beta_Sensor3 = eye(3);
            Delta_beta_Sensor4 = eye(2);
            
            Delta_beta_all = blkdiag(Delta_beta_Sensor1,Delta_beta_Sensor2,Delta_beta_Sensor3,Delta_beta_Sensor4);
            
            Q_matrix_all(ii).matrix = Delta_x_all + Delta_beta_all;
        end
       
        Sensor1_data_fusion = recalc_Posi_Celiang_Fusion(Sensor1_data_registrated,data_Fusion);
        Sensor2_data_fusion = recalc_Posi_Celiang_Fusion(Sensor2_data_registrated,data_Fusion);
        Sensor3_data_fusion = recalc_Posi_Celiang_Fusion(Sensor3_data_registrated,data_Fusion);
        Sensor4_data_fusion = recalc_Posi_Celiang_Fusion(Sensor4_data_registrated,data_Fusion);
%         Sensor1_data_fusion = Sensor1_data_fusion1 - Delta_x_Sensor1;
%         Sensor2_data_fusion = Sensor1_data_fusion2 - Delta_x_Sensor2;
%         Sensor3_data_fusion = Sensor1_data_fusion3 - Delta_x_Sensor3;
%         Sensor4_data_fusion = Sensor1_data_fusion4 - Delta_x_Sensor4;
        
        Q_invSigma_Q = 0;
        Q_invSigma_X = 0;
        for ii = 1:data_length
            AER_Sensor1 = [Sensor1_data_registrated(ii).Tar_Theta; Sensor1_data_registrated(ii).Tar_Phi; Sensor1_data_registrated(ii).Tar_R];
            AER_Sensor2 = [Sensor2_data_registrated(ii).Tar_Theta; Sensor2_data_registrated(ii).Tar_Phi; Sensor2_data_registrated(ii).Tar_R];
            AER_Sensor3 = [Sensor3_data_registrated(ii).Tar_Theta; Sensor3_data_registrated(ii).Tar_Phi; Sensor3_data_registrated(ii).Tar_R];
            AER_Sensor4 = [Sensor4_data_registrated(ii).Tar_Theta; Sensor4_data_registrated(ii).Tar_Phi];
            Xm = [AER_Sensor1; AER_Sensor2; AER_Sensor3; AER_Sensor4];
            Sensor_data_fusion_all = [Sensor1_data_fusion(ii,1:3) Sensor2_data_fusion(ii,1:3) Sensor3_data_fusion(ii,1:3) Sensor4_data_fusion(ii,1:2)].';
            
            Q_invSigma_Q = Q_invSigma_Q + Q_matrix_all(ii).matrix.' * Sigma_inv_all(ii).matrix * Q_matrix_all(ii).matrix;
            Q_invSigma_X = Q_invSigma_X + Q_matrix_all(ii).matrix.' * Sigma_inv_all(ii).matrix * (Xm - Sensor_data_fusion_all);
        end
        beta1 = pinv(Q_invSigma_Q) * Q_invSigma_X;
        beta = beta + beta1;
        %     disp('������1ƫ��(��λ������������)')
        %     disp(beta(1)/pi*180)
        %     disp(beta(2)/pi*180)
        %     disp(beta(3))
        %     disp('������2ƫ��(��λ������������)')
        %     disp(beta(4)/pi*180)
        %     disp(beta(5)/pi*180)
        %     disp(beta(6))
        %     disp('������3ƫ��(��λ������������)')
        %     disp(beta(7)/pi*180)
        %     disp(beta(8)/pi*180)
        %     disp(beta(9))
        %     disp('������4ƫ��(��λ������������)')
        %     disp(beta(10)/pi*180)
%         %     disp(beta(11)/pi*180)
%         if nn == 1
%             [Fusion_Position_WGS_Real] = Calc_Real_Position(Fusion_time,1);
%         end
%         for ii = 1: data_length
%             Error_Fusion_x(ii) = data_Fusion(ii,1) - Fusion_Position_WGS_Real(1,ii);
%             Error_Fusion_y(ii) = data_Fusion(ii,2) - Fusion_Position_WGS_Real(2,ii);
%             Error_Fusion_z(ii) = data_Fusion(ii,3) - Fusion_Position_WGS_Real(3,ii);
%         end
%         Rmse_x = sqrt(sum(Error_Fusion_x.^2)/data_length)
%         Rmse_y = sqrt(sum(Error_Fusion_y.^2)/data_length)
%         Rmse_z = sqrt(sum(Error_Fusion_z.^2)/data_length)
%         RMSE_all = sqrt(Rmse_x^2 + Rmse_y^2 + Rmse_z^2)
%         RMSE_all_1(tt) = RMSE_all;
    end
%     figure
%     plot(RMSE_all_1/1e3,'b-*','LineWidth',2.5);
%     xlabel('Number of iterations','FontSize',16);
%     ylabel('RMSE (Km)','FontSize',16);
%     set(gca,'FontSize',16)
    Period = 2*chouqv;
    Begin_Working = 1;
    Fusion_time = Period * (0:data_length-1) + Begin_Working;
    if nn == 1
        [Fusion_Position_WGS_Real] = Calc_Real_Position(Fusion_time,1);
    end
    for ii = 1: data_length
        Error_Fusion_x(ii) = data_Fusion(ii,1) - Fusion_Position_WGS_Real(1,ii);
        Error_Fusion_y(ii) = data_Fusion(ii,2) - Fusion_Position_WGS_Real(2,ii);
        Error_Fusion_z(ii) = data_Fusion(ii,3) - Fusion_Position_WGS_Real(3,ii);
    end
    Rmse_x = sqrt(sum(Error_Fusion_x.^2)/data_length)
    Rmse_y = sqrt(sum(Error_Fusion_y.^2)/data_length)
    Rmse_z = sqrt(sum(Error_Fusion_z.^2)/data_length)
    RMSE_all = sqrt(Rmse_x^2 + Rmse_y^2 + Rmse_z^2)

    dir_name = strcat('Registration_19_chouqv_',num2str(chouqv));
    mkdir(dir_name)
    data_name = ['.\',dir_name,'\Data_Registrated_' int2str(nn) '.mat'];
    save(data_name,'Sensor1_data_chouqv','Sensor2_data_chouqv','Sensor3_data_chouqv','Sensor4_data_chouqv','beta','data_Fusion','data_length','chouqv','Fusion_Position_WGS_Real',...
        'Error_Fusion_x','Error_Fusion_y','Error_Fusion_z','Rmse_x','Rmse_y','Rmse_z','RMSE_all');

end
%% �ںϺ�����û�ͼ
figure;
plot3(data_Fusion_befor_reg(1:chouqv:end,1),data_Fusion_befor_reg(1:chouqv:end,2),data_Fusion_befor_reg(1:chouqv:end,3),'LineWidth',2);
hold on
plot3(data_Fusion(1:chouqv:end,1),data_Fusion(1:chouqv:end,2),data_Fusion(1:chouqv:end,3),'r--','LineWidth',2);
hold on
plot3(Fusion_Position_WGS_Real(1,1:chouqv:end),Fusion_Position_WGS_Real(2,1:chouqv:end),Fusion_Position_WGS_Real(3,1:chouqv:end),'k.','LineWidth',8);
legend('Unregistered Trace','Registered Trace','True trajectory')
xlabel('X(m)','FontSize',16);
ylabel('Y(m)','FontSize',16);
zlabel('Z(m)','FontSize',16);
set(gca,'FontSize',14)
% set(gca,'Fontname','Times New Roman','FontSize',14)
%%��ͼXY��XZ��YZ
% XY
figure;
plot(data_Fusion_befor_reg(1:chouqv:end,1),data_Fusion_befor_reg(1:chouqv:end,2),'LineWidth',2);
hold on
plot(data_Fusion(1:chouqv:end,1),data_Fusion(1:chouqv:end,2),'r*','LineWidth',2);
hold on
plot(Fusion_Position_WGS_Real(1,1:chouqv:end),Fusion_Position_WGS_Real(2,1:chouqv:end),'k-.','LineWidth',2);
legend('Unregistered Trace','Registered Trace','True trajectory')
xlabel('X(m)','FontSize',16);
ylabel('Y(m)','FontSize',16);
set(gca,'FontSize',14)

% XZ
figure;
plot(data_Fusion_befor_reg(1:chouqv:end,1),data_Fusion_befor_reg(1:chouqv:end,3),'LineWidth',2);
hold on
plot(data_Fusion(1:chouqv:end,1),data_Fusion(1:chouqv:end,3),'r*','LineWidth',2);
hold on
plot(Fusion_Position_WGS_Real(1,1:chouqv:end),Fusion_Position_WGS_Real(3,1:chouqv:end),'k-.','LineWidth',2);
legend('Unregistered Trace','Registered Trace','True trajectory')
xlabel('X(m)','FontSize',16);
ylabel('Z(m)','FontSize',16);
set(gca,'FontSize',14)

% YZ
figure;
plot(data_Fusion_befor_reg(1:chouqv:end,2),data_Fusion_befor_reg(1:chouqv:end,3),'LineWidth',2);
hold on
plot(data_Fusion(1:chouqv:end,2),data_Fusion(1:chouqv:end,3),'r*','LineWidth',2);
hold on
plot(Fusion_Position_WGS_Real(2,1:chouqv:end),Fusion_Position_WGS_Real(3,1:chouqv:end),'k-.','LineWidth',2);
legend('Unregistered Trace','Registered Trace','True trajectory')
xlabel('Y(m)','FontSize',16);
ylabel('Z(m)','FontSize',16);
set(gca,'FontSize',14)
% figure;
% plot(Fusion_time,data_Fusion_befor_reg(1:chouqv:end,1),'LineWidth',2);
% hold on
% plot(Fusion_time,data_Fusion(1:chouqv:end,1),'r--','LineWidth',2);
% hold on
% plot(Fusion_time,Fusion_Position_WGS_Real(1,1:chouqv:end),'k.','LineWidth',8);
% legend('Unregistered Trace','Registered Trace','True trajectory')
% xlabel('Time (s)','FontSize',16);
% ylabel('X (m)','FontSize',16);
% set(gca,'FontSize',16)

%% ������1�����û�ͼ
% for m=1:data_length
%     temp = Sensor1_data_chouqv(m).Tar_Posi;
%     Sensor1_data_chouqv_Tar_Posi(m,:) = temp.';
%     temp1 = Sensor1_data_registrated(m).Tar_Posi;
%     Sensor1_data_registrated_Tar_Posi(m,:) = temp1.';
% end
    
    
% figure;
% plot3(Sensor1_data_chouqv_Tar_Posi(1:chouqv:end,1),Sensor1_data_chouqv_Tar_Posi(1:chouqv:end,2),Sensor1_data_chouqv_Tar_Posi(1:chouqv:end,3),'LineWidth',2);
% hold on
% plot3(Sensor1_data_registrated_Tar_Posi(1:chouqv:end,1),Sensor1_data_registrated_Tar_Posi(1:chouqv:end,2),Sensor1_data_registrated_Tar_Posi(1:chouqv:end,3),'r--','LineWidth',2);
% hold on
% plot3(Fusion_Position_WGS_Real(1,1:chouqv:end),Fusion_Position_WGS_Real(2,1:chouqv:end),Fusion_Position_WGS_Real(3,1:chouqv:end),'k.','LineWidth',8);
% legend('Unregistered Trace','Registered Trace','True trajectory')
% xlabel('X (m)','FontSize',16);
% ylabel('Y (m)','FontSize',16);
% zlabel('Z (m)','FontSize',16);
% set(gca,'FontSize',16)
% % figure;
% plot(0:length(RMSE_all_store)-1,RMSE_all_store,'LineWidth',2)
% xlabel('Number of iterations');
% ylabel('RMSE of all plots (m)');

