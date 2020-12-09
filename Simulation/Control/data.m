% DATA ENTRY
step_time = 0.001; % in seconds
%robot parameters
l = 0.3; % in meters
r = 0.1; % in meters
%initial positions
x1 = 0.6; % in meters
y1 = 0.8; % in meters
P1 = [ x1; y1];
theta1 = 0.15; %in radians
%final positions
x2 = 3; % in meters
y2 = 2; % in meters
P2 = [x2; y2];
theta2 = 1.2; % %in radians
%speed parameters
vT = 0.12;         %in meter/seconds
arc_omega = 0.1;  % in rad/seconds
%end of DATA ENTRY

%COMPUTATION OF THE POINTS
%line intersection point computation
data_matrix = [cos(theta1) -cos(theta2) ; sin(theta1) -sin(theta2)];
lambda_P3   = inv(data_matrix) * (P2-P1);
lambda1_P3  = lambda_P3(1);
% intersection point
P3          = P1 + lambda1_P3 * [cos(theta1) ; sin(theta1)]; 

%COMPUTATION OF MOTION SEGMENT DURATIONS
%distance traveled
s_line1     = lambda1_P3; %% length of line 1
lambda2_P3  = lambda_P3(2); %% vectoral distance from point 2 to intersection
s_line2     = abs(lambda2_P3); %% length of line 2 - ans due to negativity

%durations
duration_line1 = s_line1/vT;
duration_turn  = (theta2 -theta1) / arc_omega;
duration_line2 = s_line2/vT;
duration_total = duration_turn+ duration_line1 + duration_line2;

% cumulative transition times part by part
t1 = 0;
t2 = duration_line1;
t3 = t2+duration_turn;
t4 = duration_total;
%END OF COMPUTATION OF MOTION SEGMENT DURATIONS

array_length=ceil(duration_total/step_time )+1;
time_list=zeros(array_length,1);

% position and orientation arrays
% obtain desired positions
x_list = zeros(array_length,1);
y_list = zeros(array_length,1);
theta_list = zeros(array_length,1);
for iteration_index=1:1:array_length
    time = (iteration_index-1) *step_time;
    time_list(iteration_index) = time;

    if ((t1 <= time)  && (time <t2))
        P = P1 + time *vT *[cos(theta1) ; sin(theta1)];
        theta = theta1;
        x = P(1);
        y = P(2);
    end  
    if ((t2<=time) && (time < t3))
        theta = theta1 + (time - t2 ) * arc_omega;
        x = x_list(iteration_index -1); %% put previous position or P3 directly
        y = y_list(iteration_index -1);
    end    
    if ((t3<=time ) && (time <= t4))
        P = P3 + (time-t3)* vT * [cos(theta2); sin(theta2)];
        theta = theta2;
        x = P(1);
        y = P(2);
    end

    x_list(iteration_index) = x;
    y_list(iteration_index) = y;
    theta_list(iteration_index) = theta;
end

% obtain desired velocities
x_dot_list = zeros(array_length,1);
y_dot_list = zeros(array_length,1);
theta_dot_list = zeros(array_length,1);
for iteration_index=1:1:array_length
    
    time = (iteration_index-1) *step_time;
    
    if ((t1 <= time)  && (time <t2))
        x_dot =vT * cos(theta1);
        y_dot =vT * sin(theta1);
        theta_dot = 0;
       
    elseif ((t2<=time) && (time < t3))
        theta = theta1 + (time-t2) *arc_omega;
        x_dot = 0; %% since no pos change 0 velocities
        y_dot = 0;
        theta_dot = arc_omega;
        
    elseif ((t3<=time ) && (time <= t4))
        x_dot =vT * cos(theta2);
        y_dot =vT * sin(theta2);
        theta_dot = 0;
    end
    
    x_dot_list(iteration_index) = x_dot;
    y_dot_list(iteration_index) = y_dot;
    theta_dot_list(iteration_index) = theta_dot;
end

% new radius for the left one
r2 = r; 
r =0.095;

x_forward_list = zeros(array_length,1);
y_forward_list = zeros(array_length,1);
theta_forward_list= zeros(array_length,1);
phi1_dot_list = zeros(array_length,1);
phi2_dot_list = zeros(array_length,1);
x_dot_forward_list = zeros(array_length,1);
y_dot_forward_list = zeros(array_length,1);
theta_dot_forward_list = zeros(array_length,1);