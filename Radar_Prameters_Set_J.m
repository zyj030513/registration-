
function [Radar_Number,RadarPar] = Radar_Prameters_Set_J()
      
% 原点位置
L_rad_MISS = 103*pi/180;       % 经度，单位：rad
B_rad_MISS = 35*pi/180;        % 纬度，单位：rad
H_rad_MISS = 0;                     % 高度，单位：m

f = 1 / 298.257;
e_2 = 2 * f - f^2;
a = 6378137;

% 目标的地面原点位置
Range_L1 = 10e3;

Range_B1 = 100e3;
L_rad_AEW = 2 * asin(sin(Range_L1/a/2)/cos(B_rad_MISS)) + L_rad_MISS;
B_rad_AEW = Range_B1/a/2 + B_rad_MISS;
H_rad_AEW = 0;                     % 高度，单位：m


Radar_Number = 4;                 % 雷达数目 

Period = 2;
y_bias = 0/180*pi;
r_bias = 0/180*pi;
p_bias = 0/180*pi;

theta_bias = 5/180*pi;
phi_bias = 5/180*pi;
range_bias = 3000;

theta_error = 1/180*pi;
phi_error = 1/180*pi;
range_error = 100;
%%
RadarPar(1,1).Type = 3;                   
RadarPar(1,1).Begin_Working = 1;       
RadarPar(1,1).Slice_Interval = 0.02;      
RadarPar(1,1).Processing_Delay = 0.02;   
RadarPar(1,1).Period = Period;                 
RadarPar(1,1).y_bias = y_bias;      
RadarPar(1,1).r_bias = r_bias;      
RadarPar(1,1).p_bias = p_bias;     
RadarPar(1,1).theta_bias = theta_bias;  
RadarPar(1,1).phi_bias = phi_bias;  
RadarPar(1,1).range_bias = range_bias;           
RadarPar(1,1).theta_error = theta_error;  
RadarPar(1,1).phi_error = phi_error;     
RadarPar(1,1).range_error = range_error;        

Range_L = 50e3;
Range_B = 30e3;
RadarPar(1,1).L_rad = -2 * asin(sin(Range_L/a/2)/cos(B_rad_AEW)) + L_rad_AEW;
RadarPar(1,1).B_rad = Range_B/a/2 + B_rad_AEW;                               
RadarPar(1,1).H_rad = 40;            

%% 
RadarPar(1,2).Type = 3;                  
RadarPar(1,2).Begin_Working = 1;       
RadarPar(1,2).Slice_Interval = 0.02;     
RadarPar(1,2).Processing_Delay = 0.02;   
RadarPar(1,2).Period = Period;                 
RadarPar(1,2).y_bias = -y_bias;       
RadarPar(1,2).r_bias = -r_bias;      
RadarPar(1,2).p_bias = -p_bias;      
RadarPar(1,2).theta_bias = -theta_bias;  
RadarPar(1,2).phi_bias = -phi_bias;    
RadarPar(1,2).range_bias = -range_bias;          
RadarPar(1,2).theta_error = theta_error;   
RadarPar(1,2).phi_error = phi_error;    
RadarPar(1,2).range_error = range_error;       

Range_L = 45e3;
Range_B = 45e3;
RadarPar(1,2).L_rad = -2 * asin(sin(Range_L/a/2)/cos(B_rad_AEW)) + L_rad_AEW; 
RadarPar(1,2).B_rad = Range_B/a/2 + B_rad_AEW;                               
RadarPar(1,2).H_rad = 40;              
%% 
RadarPar(1,3).Type = 3;                  
RadarPar(1,3).Begin_Working = 1;     
RadarPar(1,3).Slice_Interval = 0.02;     
RadarPar(1,3).Processing_Delay = 0.02; 
RadarPar(1,3).Period = Period;              
RadarPar(1,3).y_bias = y_bias;      
RadarPar(1,3).r_bias = r_bias;      
RadarPar(1,3).p_bias = p_bias;      
RadarPar(1,3).theta_bias = theta_bias;  
RadarPar(1,3).phi_bias = phi_bias;    
RadarPar(1,3).range_bias = range_bias;          
RadarPar(1,3).theta_error = theta_error;  
RadarPar(1,3).phi_error = phi_error;   
RadarPar(1,3).range_error = range_error;      

Range_L = 25e3;
Range_B = 30e3;
RadarPar(1,3).L_rad = -2 * asin(sin(Range_L/a/2)/cos(B_rad_AEW)) + L_rad_AEW;  
RadarPar(1,3).B_rad = Range_B/a/2 + B_rad_AEW;                                
RadarPar(1,3).H_rad = 40;               
%% 
RadarPar(1,4).Type = 3;               
RadarPar(1,4).Begin_Working = 1;     
RadarPar(1,4).Slice_Interval = 0.02;    
RadarPar(1,4).Processing_Delay = 0.02;  
RadarPar(1,4).Period = Period;               
RadarPar(1,4).y_bias = -y_bias;      
RadarPar(1,4).r_bias = -r_bias;      
RadarPar(1,4).p_bias = -p_bias;      
RadarPar(1,4).theta_bias = -theta_bias; 
RadarPar(1,4).phi_bias = -phi_bias;  
RadarPar(1,4).range_bias = -range_bias;     
RadarPar(1,4).theta_error = theta_error/2; 
RadarPar(1,4).phi_error = phi_error/2;     
RadarPar(1,4).range_error = range_error/2;       

Range_L = 50e3;
Range_B = -20e3;
RadarPar(1,4).L_rad = -2 * asin(sin(Range_L/a/2)/cos(B_rad_AEW)) + L_rad_AEW; 
RadarPar(1,4).B_rad = Range_B/a/2 + B_rad_AEW;                              
RadarPar(1,4).H_rad = 40;              





