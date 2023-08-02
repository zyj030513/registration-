function Phi_output = Phi_cal(delta_T)
T = delta_T;
Phi1 = [1  T  T^2/2;
        0  1  T;
        0  0  1      ];
    
Phi_output = [Phi1       zeros(3,3) zeros(3,3);
              zeros(3,3) Phi1       zeros(3,3);
              zeros(3,3) zeros(3,3) Phi1        ];