function drawJointVelocities(jointVelocities, unit, text)
    figure;
    hold on;
    plot(jointVelocities(:,1));
    plot(jointVelocities(:,2));
    plot(jointVelocities(:,3));
    plot(jointVelocities(:,4));
    plot(jointVelocities(:,5));
    plot(jointVelocities(:,6));
    title(text);
    xlabel('Time (s)'); 
    ylabel('Velocity (' +unit);
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end