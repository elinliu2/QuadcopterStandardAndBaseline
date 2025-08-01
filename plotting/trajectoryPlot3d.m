function trajectoryPlot3d(time, stateProgression)
    figure(1);
    hold off
    x = stateProgression(1, :);
    y = stateProgression(2, :);
    z = stateProgression(3, :);
    
    scatter3(x, y, z, 10, time,'filled');
    hold on
    title("3D Plot");
    xlabel("x");
    ylabel("y");
    zlabel("z");
end