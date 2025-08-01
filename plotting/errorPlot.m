function errorPlot(time, error)
    figure(2);
    hold off
    plot(time, error);
    hold on
    title("RMSE over Time");
    xlabel("time [s]");
    ylabel("RMSE");
end

