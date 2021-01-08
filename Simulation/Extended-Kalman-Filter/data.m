clear all;close all;

%sampling time
delta_t = 0.001;

%plant parameters
r = 0.1; l = 0.6;

%noise parameters to use in simulation
Q = [0.001 0 0; 0 0.001 0; 0 0 0.001];
R = [0.015 0 0; 0 0.1 0 ; 0 0 0.02];

% control parameters
u1_amplitude = 0.1;  u1_frequency = 0.1;  u1_offset = 0;
u2_amplitude = 0.12; u2_frequency = 0.02; u2_offset = 0;

%- Initial Values
%initial coveriance matrix
P_k_minus_1 = [2 0 0; 0 2 0; 0 0 2];
% Initial state vector
x_initial = [1;3;pi/6];
x = x_initial;
x_i_1 = x_initial(1);
x_i_2 = x_initial(2);
x_i_3 = x_initial(3);
 
A = eye(3);
%declaration of h jacobian
delh_delx_initial = [x_i_1/sqrt(x_i_1^2 + x_i_2^2)  x_i_2/sqrt(x_i_1^2 + x_i_2^2)   0;
                     -x_i_2/sqrt(x_i_1^2 + x_i_2^2) x_i_1/sqrt(x_i_1^2 + x_i_2^2)   0;
                     0                              0                               1];
H = delh_delx_initial;

%declaration of previous state information
x_hat_k_minus_1 = x_initial;
K_k = P_k_minus_1 * H'* inv(H*P_k_minus_1*H' + R);