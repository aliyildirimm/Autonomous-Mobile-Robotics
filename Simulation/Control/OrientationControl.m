% FORWARD KINEMATICS FOR CROSSCHECK FOR CARTESIAN POSITIONS BY USING WHEEL SPEEDS
% FIRSTLY OBTAIN WHEEL SPEEDS

current_x =x1;
current_y =y1;
current_theta = theta1;

for iteration_index=1:1:array_length

   theta_error = theta_list(iteration_index) - current_theta;
   
   %right 
   phi1_dot = cos(theta_list(iteration_index))*x_dot_list(iteration_index)/r ...
            + sin(theta_list(iteration_index))*y_dot_list(iteration_index)/r ...
            + l*theta_dot_list(iteration_index)/r + Kv_theta * theta_error;
   %left
   phi2_dot = cos(theta_list(iteration_index))*x_dot_list(iteration_index)/r ...
            + sin(theta_list(iteration_index))*y_dot_list(iteration_index)/r ...
            - l *theta_dot_list(iteration_index)/r - Kv_theta * theta_error;
        
   phi1_dot_list(iteration_index) = phi1_dot;
   phi2_dot_list(iteration_index) = phi2_dot;

   pose_forward_dot = [ cos(current_theta) -sin(current_theta) 0;
                        sin(current_theta)  cos(current_theta) 0;
                        0                   0                  1] * ...
                      [r*phi1_dot_list(iteration_index)/2 + r2*phi2_dot_list(iteration_index)/2;
                       0;
                       r*phi1_dot_list(iteration_index)/(2*l) - r2*phi2_dot_list(iteration_index)/(2*l)];

   x_dot_forward_list(iteration_index) = pose_forward_dot(1);
   y_dot_forward_list(iteration_index) = pose_forward_dot(2);
   theta_dot_forward_list(iteration_index) = pose_forward_dot(3);
   
   x_forward_list(iteration_index) = current_x;
   y_forward_list(iteration_index) = current_y;
   theta_forward_list(iteration_index) = current_theta;
   
   current_x = current_x + step_time * x_dot_forward_list(iteration_index);
   current_y = current_y + step_time * y_dot_forward_list(iteration_index);
   current_theta = current_theta + step_time * pose_forward_dot(3);
end
% position check
figure('Units','normalized','Position',[0 0 1 1]); 
    plot(x_forward_list,y_forward_list, 'r')
    hold
    plot(x_list,y_list,'k')
    plot(P1(1),P1(2),'ro')
    plot(P2(1),P2(2),'bo')
    plot(P3(1),P3(2),'ko')

    axis([0 x2+1 0 y2+1])
    xlabel('x [m]')
    ylabel('y [m]')
    grid
    sgtitle(['Orientation Control with Kv Theta: ', num2str(Kv_theta)]);
    legend('Obtained Position','Reference Position')
    saveas(gcf,"./results/"+num2str(q),'jpg');

figure('Units','normalized','Position',[0 0 1 1]);
    subplot(3,2,1)
        plot(time_list,x_forward_list, 'k')
        hold on
        plot(time_list, x_list, 'r')
        xlabel('Time [s]')
        ylabel('x [m]')
        legend('Obtained Position in X','Reference Position in X', 'Location','northwest')

    subplot(3,2,3)
        plot(time_list, y_forward_list, 'k')
        hold on
        plot(time_list, y_list, 'r')
        xlabel('Time [s]')
        ylabel('y [m]')
        legend('Obtained Position in Y','Reference Position in Y', 'Location','northwest')

    subplot(3,2,5)
        plot(time_list, theta_forward_list, 'k')
        hold on
        plot(time_list, theta_list, 'r')
        xlabel('Time [s]')
        ylabel('theta [rad]')
        legend('Obtained Orientation','Reference Orientation', 'Location','northwest')

    subplot(3,2,2)
        plot(time_list,x_dot_forward_list, 'k')
        hold on
        plot(time_list, x_dot_list, 'r')
        xlabel('Time [s]')
        ylabel('x dot [m/s]')
        legend('Obtained Velocity','Reference Velocity', 'Location','northwest')

    subplot(3,2,4)
        plot(time_list, y_dot_forward_list, 'k')
        hold on
        plot(time_list, y_dot_list, 'r')
        xlabel('Time [s]')
        ylabel('y dot [m/s]')
        legend('Obtained Velocity','Reference Velocity', 'Location','northwest')

    subplot(3,2,6)
        plot(time_list, theta_dot_forward_list, 'k')
        hold on
        plot(time_list, theta_dot_list, 'r')
        xlabel('Time [s]')
        ylabel('theta dot [rad/s]')
        legend('Obtained Orientation Speed','Reference Orientation Speed', 'Location','northwest')
    
    sgtitle(['Orientation Control Effects on Positions and Velocities with Kv Theta: ', num2str(Kv_theta)]);
    saveas(gcf,"./results/"+ num2str(q + 1),'jpg');
    
figure('Units','normalized','Position',[0 0 1 1]);
    plot(time_list,phi1_dot_list, 'k')
    hold on
    plot(time_list,phi2_dot_list, 'r')
    xlabel('Time [s]')
    ylabel('Wheel velocities [rad/s]')
    legend('Wheel 1 Speed','Wheel 2 Speed')
  sgtitle(['Wheel Speeds with Kv Theta: ', num2str(Kv_theta)]);
  saveas(gcf,"./results/"+ num2str(q + 2) ,'jpg');
        

