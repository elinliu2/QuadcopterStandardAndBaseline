function controlEffortPlot(time, controlEffort)
    figure(3);
    hold off
    plot(time, controlEffort);
    hold on
    title("Control Effort over Time");
    xlabel("time [s]");
    ylabel("Control Effort");
end