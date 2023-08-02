function Q_output = Q_cal(delta_T)
T = delta_T;
Q1 = [T^5/20 T^4/8 T^3/6;
      T^4/8  T^3/3 T^2/2;
      T^3/6  T^2/2 T     ]; 
% Q1 = T*[T^4/4 T^3/2 T^2/2;
%         T^3/2 T^2   T;
%         T^2/2 T     1    ]; 
Q_output = [Q1         zeros(3,3) zeros(3,3);
            zeros(3,3) Q1         zeros(3,3);
            zeros(3,3) zeros(3,3) Q1        ];
% Q_output = 1e1 * Q_output;
Q_output = 1e-1 * Q_output;