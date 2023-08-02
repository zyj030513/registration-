function Sigma_cov = calc_covariancematrix_R(Rinv_matrix_blkdiag,Num)
% 从Rinv_matrix_blkdiag中提取分块矩阵，获得各子噪声协方差逆矩阵
Sum_Rinv = zeros(3,3);
for ii = 1:Num
    Rinv_matrix(ii).matrix = Rinv_matrix_blkdiag(3*ii-2:3*ii,3*ii-2:3*ii);
    Sum_Rinv = Sum_Rinv + Rinv_matrix(ii).matrix;
    R_matrix(ii).matrix = inv(Rinv_matrix(ii).matrix);
end
ISum_Rinv = inv(Sum_Rinv);

if Num > 1
    Sigma_cov1 = zeros(3*Num,3*Num);
    Sigma_cov2 = zeros(3*Num,3*Num);
    % 先计算出对角线外的上三角矩阵
    for ii = 1:Num-1
        for jj = ii+1:Num
            aa = zeros(3,3);
            for nn = 1:Num
                if nn == ii
                    aa = aa + (eye(3) - ISum_Rinv * Rinv_matrix(nn).matrix) * R_matrix(nn).matrix * (-ISum_Rinv * Rinv_matrix(nn).matrix).';
                elseif nn == jj
                    aa = aa + (-ISum_Rinv * Rinv_matrix(nn).matrix) * R_matrix(nn).matrix * (eye(3) - ISum_Rinv * Rinv_matrix(nn).matrix).';
                else
                    aa = aa + (-ISum_Rinv * Rinv_matrix(nn).matrix) * R_matrix(nn).matrix * (-ISum_Rinv * Rinv_matrix(nn).matrix).';
                end
            end 
            Cov_ij = aa;
            Sigma_cov1(3*ii-2:3*ii,3*jj-2:3*jj) = Cov_ij;
        end
    end
    % 再计算对角线元素
    for ii = 1:Num
        aa = zeros(3,3);
        for nn = 1:Num
            if nn == ii
                aa = aa + (eye(3) - ISum_Rinv * Rinv_matrix(nn).matrix) * R_matrix(nn).matrix * (eye(3) - ISum_Rinv * Rinv_matrix(nn).matrix).';
            else
                aa = aa + (-ISum_Rinv * Rinv_matrix(nn).matrix) * R_matrix(nn).matrix * (-ISum_Rinv * Rinv_matrix(nn).matrix).';
            end
        end
        Cov_ii = aa;
        Sigma_cov2(3*ii-2:3*ii,3*ii-2:3*ii) = Cov_ii;
    end
    % 得到所有元素
    Sigma_cov = Sigma_cov1 + Sigma_cov1.' + Sigma_cov2;
else
    Sigma_cov = inv(Rinv_matrix_blkdiag);
end



