function drawJointAccelerations(jointAccelerations, unit, text)
    figure;
    hold on;
    plot(jointAccelerations(:,1));
    plot(jointAccelerations(:,2));
    plot(jointAccelerations(:,3));
    plot(jointAccelerations(:,4));
    plot(jointAccelerations(:,5));
    plot(jointAccelerations(:,6));
    title(text);
    xlabel('Time (s)'); 
    ylabel('Acceleration ('+unit);
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end