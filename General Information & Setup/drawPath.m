function drawPath(poses, text)
    figure;
    line(poses(:,1), poses(:,2),poses(:,3));
    view(3);
    title(text)
    xlabel('x-axis (m)'); 
    ylabel('y-axis (m)');
    zlabel('z-axis (m)');
end



