clear all;close all;

% run data.m file
data

%record arrays
% real output states:
y_list      = [];
% estimated output states:
y_hat_list  = [];
% control inputs:
u_list      = [];
% Kalman Gain List:
K_k_list    = [];
% Covariance Matrix:
P_k_list    = [];

for k=1:20000
    %- DESIRED INPUT
        %-control vector update according to desired reference signal
        u_k = [u1_offset+ u1_amplitude * sin(2*pi*u1_frequency * k * delta_t);
               u2_offset+ u2_amplitude * sin(2*pi*u2_frequency * k * delta_t)];
       
    %-1ST STEP: PREDICTION UPDATE
        % current predicted states updated (through mathematical model) 
        x_hat_minus_k = x_hat_k_minus_1 +...
                        [delta_t* 0.5*r*(u_k(1) + u_k(2))* cos(x_hat_k_minus_1(3));
                         delta_t* 0.5*r*(u_k(1) + u_k(2))* sin(x_hat_k_minus_1(3));
                         delta_t* 0.5*(r/l) * (u_k(1) - u_k(2))];
        % Jacobian f Calculation with new predictions 
        % to obtain Prediction Covariance Matrix:
        A = [ 1 0 -delta_t*0.5*r*(u_k(1)+u_k(2))*sin(x_hat_k_minus_1(3));
              0 1 delta_t*0.5*r*(u_k(1)+u_k(2))*cos(x_hat_k_minus_1(3));
              0 0 1];
        % new Prediction Covariance Matrix      
        P_minus_k = A*P_k_minus_1*A' + Q;
    
    %- 2ND STEP: NECESSITIES FOR THE CORRECTION STEP:
    % For Correction Step Kalman Gain is needed:        
        % To obtain Kalman Gain 3x3 jacobian H matrix
        H =[x_hat_k_minus_1(1) /sqrt(x_hat_k_minus_1(1)^2 + x_hat_k_minus_1(2)^2)...
            x_hat_k_minus_1(2) /sqrt(x_hat_k_minus_1(1)^2 + x_hat_k_minus_1(2)^2)...
            0;
           -x_hat_k_minus_1(2) /sqrt(x_hat_k_minus_1(1)^2 + x_hat_k_minus_1(2)^2)...
            x_hat_k_minus_1(1) /sqrt(x_hat_k_minus_1(1)^2 + x_hat_k_minus_1(2)^2)...
            0;
            0 0 1 ];
             
         %- Kalman Gain Calculation:
         K_k = P_minus_k * H' * inv(H*P_minus_k*H'+R);
     
    %- 3RD STEP: REAL STATES AND MEASUREMENTS FROM SENSORS(ASSUMPTION)
        % plant simulation equation
        % previous x value is updated through some noise to
        % simulate real life example
        x = x + [delta_t* 0.5*r*(u_k(1) + u_k(2))* cos(x(3));
                 delta_t* 0.5*r*(u_k(1) + u_k(2))* sin(x(3));
                 delta_t* 0.5*(r/l) * (u_k(1)- u_k(2))] +...
                [sqrt(Q(1,1)) * randn; sqrt(Q(2,2)) * randn; sqrt(Q(3,3)) * randn];
     
        % measurement simulation
        % measured distances with the added noise to simulate real life example
        % since sensor values will have some noise. 
        d           = sqrt(x(1)^2+ x(2)^2);
        alpha       = atan2(x(2),x(1));
        d_noisy     = d + sqrt(R(1,1))* randn;
        alpha_noisy = alpha +  sqrt(R(2,2))* randn;
        theta_noisy = x(3) +  sqrt(R(3,3))* randn;
        
        % Measurement State at time step k:
        z_k = [d_noisy; alpha_noisy; theta_noisy];
        
     %- FINAL STEP: CORRECTION
         x_hat_k =  x_hat_minus_k +...
                    K_k*(z_k -[sqrt(x_hat_minus_k(1)^2 + x_hat_minus_k(2)^2);
                               atan2(x_hat_minus_k(2),x_hat_minus_k(1));
                               x_hat_minus_k(3)]);
     
     %Error Covariance Matrix Update
     P_k = (eye(3) -K_k*H)*P_minus_k;
     
     %variables for the next cycle
     P_k_minus_1 = P_k;
     x_hat_k_minus_1 = x_hat_k;
     
     %record the obtained values in array
     y_list     = [y_list;x'];
     y_hat_list = [y_hat_list;x_hat_k'];
     u_list     = [u_list;u_k'];
     K_k_list   = [K_k_list; K_k(1,1)];
end

plots