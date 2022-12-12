function drawJointPositions(jointPositions, text)
    figure;
    hold on;
    plot(jointPositions(:,1));
    plot(jointPositions(:,2));
    plot(jointPositions(:,3));
    plot(jointPositions(:,4));
    plot(jointPositions(:,5));
    plot(jointPositions(:,6));
    title(text)
    xlabel('Time (s)'); 
    ylabel('Joint angles (radians)');
    legend({'Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'},'Location','southwest');
    hold off;
end