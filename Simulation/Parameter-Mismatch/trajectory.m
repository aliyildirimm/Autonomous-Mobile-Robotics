close all;clear all;clc;
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

%Question 1:
%PART A) GEOMETRIC CONSTRUCTION OF THE CARTESIAN REFERENCE CURVE
%data storage dimension
array_length=ceil(duration_total/step_time )+1;
% position and orientation arrays
x_list = zeros(array_length,1);
y_list = zeros(array_length,1);
theta_list = zeros(array_length,1);
time_list=zeros(array_length,1);

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

figure
    plot(x_list,y_list)
    title("Reference Cartesian Curve")
    hold

    plot(P1(1),P1(2),'ro')
    plot(P2(1),P2(2),'bo')
    plot(P3(1),P3(2),'ko')
    legend('Reference Curve','Initial','Final','Intersection')

    axis([0 x2+1 0 y2+1])
    xlabel('x [m]')
    ylabel('y [m]')
    grid

figure
    subplot(3,1,1)
        sgtitle("Reference x,y,theta versus Time")
        plot(time_list,x_list)
        xlabel('Time [s]')
        ylabel('x [m]')

    subplot(3,1,2)
        plot(time_list,y_list)
        xlabel('Time [s]')
        ylabel('y [m]')

    subplot(3,1,3)
        plot(time_list,theta_list)
        xlabel('Time [s]')
        ylabel('theta [rad]')
%END OF PART A) GEOMETRIC CONSTRUCTION OF THE CARTESIAN REFERENCE CURVE

%PART B) GEOMETRIC CONSTRUCTION OF THE REFERENCE CARTESIAN SPEEDS
% arrays for rate of change of positions and orientation in step time
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
figure 
    subplot(3,1,1)
        plot(time_list,x_dot_list)
        xlabel('Time [s]')
        ylabel('x dot [m/s]')

    subplot(3,1,2)
        plot(time_list,y_dot_list)
        xlabel('Time [s]')
        ylabel('y dot [m/s]')

    subplot(3,1,3)
        plot(time_list,theta_dot_list)
        xlabel('Time [s]')
        ylabel('theta dot [rad/s]')

    sgtitle("Reference Speeds")
%END OF PART B) GEOMETRIC CONSTRUCTION OF THE REFERENCE CARTESIAN SPEEDS


%PART C) INVERSE KINEMATICS FOR WHEEL SPEEDS
% wheel speed arrays with respect to time
phi1_dot_list = zeros(array_length,1);
phi2_dot_list = zeros(array_length,1);

for iteration_index=1:1:array_length
    
   phi1_dot = cos(theta_list(iteration_index))*x_dot_list(iteration_index)/r ...
            + sin(theta_list(iteration_index))*y_dot_list(iteration_index)/r ...
            + l*theta_dot_list(iteration_index)/r;
   
   phi2_dot = cos(theta_list(iteration_index))*x_dot_list(iteration_index)/r ...
            + sin(theta_list(iteration_index))*y_dot_list(iteration_index)/r ...
            - l *theta_dot_list(iteration_index)/r;
   phi1_dot_list(iteration_index) = phi1_dot;
   phi2_dot_list(iteration_index) = phi2_dot;
   
end

figure
    subplot(2,1,1)
        plot(time_list,phi1_dot_list)
        xlabel('Time [s]')
        ylabel('Phi1 dot [rad/s]')
        
    subplot(2,1,2)
        plot(time_list,phi2_dot_list)
        xlabel('Time [s]')
        ylabel('Phi2 dot [rad/s]')
        sgtitle("Wheel Speeds Computed Through Inverse Kinematics")
%END OF PART C) INVERSE KINEMATICS FOR WHEEL SPEEDS


%PART D) FORWARD KINEMATICS FOR CROSSCHECK FOR CARTESIAN POSITIONS 
%        BY USING WHEEL SPEEDS
% FIRSTLY OBTAIN WHEEL SPEEDS
x_dot_forward_list = zeros(array_length,1);
y_dot_forward_list = zeros(array_length,1);
theta_dot_forward_list = zeros(array_length,1);

current_theta = theta1;

for iteration_index=1:1:array_length
    
   pose_forward_dot = [cos(current_theta) -sin(current_theta) 0;
                       sin(current_theta)  cos(current_theta) 0;
                       0                   0                  1] * ...
                      [r*phi1_dot_list(iteration_index)/2 + r*phi2_dot_list(iteration_index)/2;
                       0;
                       r*phi1_dot_list(iteration_index)/(2*l) - r*phi2_dot_list(iteration_index)/(2*l)];
   
   x_dot_forward_list(iteration_index) = pose_forward_dot(1);
   y_dot_forward_list(iteration_index) = pose_forward_dot(2);
   theta_dot_forward_list(iteration_index) = pose_forward_dot(3);
   
   current_theta = current_theta + step_time * pose_forward_dot(3);
end

% SECONDLY CARTESIAN SPEEDS INTEGRATION FOR CARTESIAN POSITON CALCULATION
x_forward_list = zeros(array_length,1);
y_forward_list = zeros(array_length,1);
theta_forward_list= zeros(array_length,1);

current_x =x1;
current_y =y1;
current_theta = theta1;
for iteration_index=1:1:array_length
    
   x_forward = current_x;
   y_forward = current_y;
   theta_forward= current_theta;
   
   x_forward_list(iteration_index) = x_forward;
   y_forward_list(iteration_index) = y_forward;
   theta_forward_list(iteration_index) = theta_forward;
   
   current_x     = current_x + step_time * x_dot_forward_list(iteration_index);
   current_y     = current_y + step_time * y_dot_forward_list(iteration_index);
   current_theta = current_theta + step_time * theta_dot_forward_list(iteration_index);
end

figure 
    plot(x_forward_list,y_forward_list,'k')
    hold
    plot(x_list,y_list,'r')
    plot(P1(1),P1(2),'ro')
    plot(P2(1),P2(2),'bo')
    plot(P3(1),P3(2),'ko')
    axis([0 x2+1 0 y2+1])
    xlabel('x [m]')
    ylabel('y [m]')
    grid
    legend('Forward Kinematics','Reference Curve')
    sgtitle("Forward Kinematics Cartesian Trajectory Crooscheck")
    
figure 
    subplot(3,1,1)
        plot(time_list,x_forward_list,'k',time_list,x_list,'r')
        xlabel('Time [s]')
        ylabel('x [m]')
        legend('Forward Kinematics','Reference Curve')
        
    subplot(3,1,2)
        plot(time_list,y_forward_list,'k',time_list,y_list,'r')
        xlabel('Time [s]')
        ylabel('y [m]')
        legend('Forward Kinematics','Reference Curve')
    
    subplot(3,1,3)
        plot(time_list,theta_forward_list,'k',time_list,theta_list,'r')
        xlabel('Time [s]')
        ylabel('theta [rad]')
        legend('Forward Kinematics','Reference Curve')
    
    sgtitle("Forward Kinematics Crooscheck For Each Cartesian Trajectory (x,y,\theta)")
%END OF PART D) CARTESIAN SPEED INTEGRATION CROOSCHECK FOR CARTESIAN POSITON

% QUESTION 2: INTRODUCTION OF A MODEL PARAMETER MISMATCH
% FORWARD KINEMATICS FOR CROSSCHECK FOR CARTESIAN POSITIONS BY USING WHEEL SPEEDS
% FIRSTLY OBTAIN WHEEL SPEEDS
% new radius for the left one
r2 = 0.1;
r = 0.095;
x_dot_forward_list = zeros(array_length,1);
y_dot_forward_list = zeros(array_length,1);
theta_dot_forward_list = zeros(array_length,1);

current_theta = theta1;

for iteration_index=1:1:array_length
    
   pose_forward_dot = [ cos(current_theta) -sin(current_theta) 0;
                        sin(current_theta)  cos(current_theta) 0;
                        0                   0                  1] * ...
                      [r*phi1_dot_list(iteration_index)/2 + r2*phi2_dot_list(iteration_index)/2;
                       0;
                       r*phi1_dot_list(iteration_index)/(2*l) - r2*phi2_dot_list(iteration_index)/(2*l)];
   
   x_dot_forward_list(iteration_index) = pose_forward_dot(1);
   y_dot_forward_list(iteration_index) = pose_forward_dot(2);
   theta_dot_forward_list(iteration_index) = pose_forward_dot(3);
   
   current_theta = current_theta + step_time * pose_forward_dot(3);
end

% SECONDLY CARTESIAN SPEEDS INTEGRATION FOR CARTESIAN POSITON CALCULATION
x_forward_list = zeros(array_length,1);
y_forward_list = zeros(array_length,1);
theta_forward_list= zeros(array_length,1);

current_x =x1;
current_y =y1;
current_theta = theta1;
for iteration_index=1:1:array_length
    
   x_forward     = current_x;
   y_forward     = current_y;
   theta_forward = current_theta;
   
   x_forward_list(iteration_index)      = x_forward;
   y_forward_list(iteration_index)      = y_forward;
   theta_forward_list(iteration_index)  = theta_forward;
   
   current_x     = current_x + step_time * x_dot_forward_list(iteration_index);
   current_y     = current_y + step_time * y_dot_forward_list(iteration_index);
   current_theta = current_theta + step_time * theta_dot_forward_list(iteration_index);
end

figure
    plot(x_forward_list,y_forward_list,'r')
    hold
    plot(x_list,y_list,'k')
    plot(P1(1),P1(2),'ro')
    plot(P2(1),P2(2),'bo')
    plot(P3(1),P3(2),'ko')

    axis([0 x2+1 0 y2+1])
    xlabel('x [m]')
    ylabel('y [m]')
    grid
    legend('Parameter Mismatch','Initial','Final','Intersection')
    sgtitle("Position Differences for Obtained vs Reference Trajectory in 2D ")
    
figure 
    subplot(3,1,1)
        plot(time_list,x_forward_list,'k',time_list,x_list,'r')
        xlabel('Time [s]')
        ylabel('x [m]')

    subplot(3,1,2)
        plot(time_list,y_forward_list,'k',time_list,y_list,'r')
        xlabel('Time [s]')
        ylabel('y [m]')

    subplot(3,1,3)
        plot(time_list,theta_forward_list,'k',time_list,theta_list,'r')
        xlabel('Time [s]')
        ylabel('theta [rad]')
        sgtitle('Position and Orientation Differences for Obtained vs Reference Trajectory in 1D ')
