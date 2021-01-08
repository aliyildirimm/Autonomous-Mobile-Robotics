figure('WindowState','maximize')
    subplot(2,1,1)
        plot(u_list(:,1))
        xlabel("Time index")
        ylabel("Control Input 1")
    subplot(2,1,2)
        plot(u_list(:,2))
        xlabel("Time index")
        ylabel("Control Input 2")
    sgtitle('Control Inputs u_{k1}and u_{k2} For Wheels 1 and 2 ')
    %saveas(gcf, './Results/Control Inputs', 'jpg');

figure('WindowState','maximize')
    subplot(3,1,1)
        plot(y_list(:,1),'r')
        xlabel("Time index")
        ylabel("Real Position in x direction")
    subplot(3,1,2)
        plot(y_hat_list(:,1),"k")
        xlabel("Time index")
        ylabel("Estimated Position in x direction")
    subplot(3,1,3)
        plot(y_list(:,1),'r')
        hold
        plot(y_hat_list(:,1),'k')
        xlabel("Time index")
        ylabel("X-Position")
        legend('Correct','Estimated')
    sgtitle('Correct Position vs Estimation in x direction')
    %saveas(gcf, './Results/X-dir', 'jpg');

figure('WindowState','maximize')
    plot(y_list(:,1),'r')
    hold
    plot(y_hat_list(:,1),'k')
    xlabel("Time index")
    ylabel("X-Position")
    legend('Correct','Estimated')
    title('Correct Position vs Estimation in x direction Closer Look')
    %saveas(gcf, './Results/X-dir-closer', 'jpg');
    
figure('WindowState','maximize')
    subplot(3,1,1)
        plot(y_list(:,2),'r')
        xlabel("Time index")
        ylabel("Real Position in y direction")
    subplot(3,1,2)
        plot(y_hat_list(:,2),"k")
        xlabel("Time index")
        ylabel("Estimated Position in y direction")
    subplot(3,1,3)
        plot(y_list(:,2),'r')
        hold
        plot(y_hat_list(:,2),'k')
        xlabel("Time index")
        ylabel("Y-Position")
        legend('Correct','Estimated')
    sgtitle('Correct Position vs Estimation in y direction')
    %saveas(gcf, './Results/Y-dir', 'jpg');

figure('WindowState','maximize')
    plot(y_list(:,2),'r')
    hold
    plot(y_hat_list(:,2),'k')
    xlabel("Time index")
    ylabel("Y-Position")
    legend('Correct','Estimated')
    title('Correct Position vs Estimation in y direction Closer Look')
    %saveas(gcf, './Results/Y-dir-closer', 'jpg');

figure('WindowState','maximize')
    subplot(3,1,1)
        plot(y_list(:,3),'r')
        xlabel("Time index")
        ylabel("Real \Theta")
    subplot(3,1,2)
        plot(y_hat_list(:,3),"k")
        xlabel("Time index")
        ylabel("Estimated \theta")
    subplot(3,1,3)
        plot(y_list(:,3),'r')
        hold
        plot(y_hat_list(:,3),'k')
        xlabel("Time index")
        ylabel("\theta")
        legend('Correct','Estimated')
    sgtitle('Correct \theta vs Estimated \theta')
    saveas(gcf, './Results/theta-dir', 'jpg');

figure('WindowState','maximize')
    plot(y_list(:,3),'r')
    hold
    plot(y_hat_list(:,3),'k')
    xlabel("Time index")
    ylabel("\theta")
    legend('Correct','Estimated')
    title('Correct \theta vs Estimated \theta Closer Look')
    saveas(gcf, './Results/theta-dir-closer', 'jpg');
    
figure('WindowState','maximize')
    plot(K_k_list(:,1),'b')
    xlabel("Time index")
    ylabel("K_k")
    title("Kalman Gain K_k")
    saveas(gcf, './Results/Kalman', 'jpg');