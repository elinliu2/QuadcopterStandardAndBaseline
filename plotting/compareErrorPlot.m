function compareErrorPlot(mpc_time, mpc_error, bk_time, bk_error)
    figure(2);
    hold off
    plot(mpc_time, mpc_error);
    hold on
    plot(bk_time, bk_error);
    title("RMSE over Time");
    xlabel("time [s]");
    ylabel("RMSE");
    legend("MPC", "Integral BK")
end

