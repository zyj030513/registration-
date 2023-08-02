function Sigma_cov = calc_covariancematrix_R_R16(R_matrix_spherical_all,Rinv_matrix_84_all,HA_x_matrix_Sensor_all,Num)
% ��Rinv_matrix_blkdiag����ȡ�ֿ���󣬻�ø�������Э���������
Sum_Rinv = zeros(3,3);
for ii = 1:Num
    R_matrix_spherical(ii).matrix = R_matrix_spherical_all(3*ii-2:3*ii,3*ii-2:3*ii);
    Rinv_matrix_84(ii).matrix = Rinv_matrix_84_all(3*ii-2:3*ii,3*ii-2:3*ii);
    HA_x_matrix_Sensor_n(ii).matrix = HA_x_matrix_Sensor_all(3*ii-2:3*ii,3*ii-2:3*ii);
    
    Sum_Rinv = Sum_Rinv + Rinv_matrix_84(ii).matrix;
    R_matrix(ii).matrix = inv(Rinv_matrix_84(ii).matrix);
end
ISum_Rinv = inv(Sum_Rinv);

if Num > 1
    Sigma_cov1 = zeros(3*Num,3*Num);
    Sigma_cov2 = zeros(3*Num,3*Num);
    Sigma_cov3 = zeros(3*Num,3*Num);
    % �ȼ�����Խ�����������Ǿ���
    for ii = 1:Num-1
        for jj = ii+1:Num
            aa = zeros(3,3);
            for nn = 1:Num
                if nn == ii
                    aa = aa + (eye(3) - HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (-HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                elseif nn == jj
                    aa = aa + (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (eye(3) - HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                else
                    aa = aa + (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (-HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                end
            end
            Cov_ij = aa;
            Sigma_cov1(3*ii-2:3*ii,3*jj-2:3*jj) = Cov_ij;
        end
    end
    % �ټ���Խ���Ԫ��
    for ii = 1:Num
        aa = zeros(3,3);
        for nn = 1:Num
            if nn == ii
                aa = aa + (eye(3) - HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (eye(3) - HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
            else
                aa = aa + (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
            end
        end
        Cov_ii = aa;
        Sigma_cov2(3*ii-2:3*ii,3*ii-2:3*ii) = Cov_ii;
    end
    % ��������Խ�����������Ǿ���
    for jj = 1:Num-1
        for ii = jj+1:Num
            aa = zeros(3,3);
            for nn = 1:Num
                if nn == ii
                    aa = aa + (eye(3) - HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (-HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                elseif nn == jj
                    aa = aa + (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (eye(3) - HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                else
                    aa = aa + (-HA_x_matrix_Sensor_n(ii).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix) * R_matrix_spherical(nn).matrix * (-HA_x_matrix_Sensor_n(jj).matrix * ISum_Rinv * Rinv_matrix_84(nn).matrix).';
                end
            end
            Cov_ij = aa;
            Sigma_cov3(3*ii-2:3*ii,3*jj-2:3*jj) = Cov_ij;
        end
    end
    % �õ�����Ԫ��
%     Sigma_cov = Sigma_cov1 + Sigma_cov2 + Sigma_cov3;
    Sigma_cov = Sigma_cov1 + Sigma_cov2 + Sigma_cov1.';
else
    disp('����������С��2')
end
