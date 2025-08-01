function compareControlEffortPlot(mpc_time, mpc_controlEffort, bk_time, bk_controlEffort)
    figure(3);
    hold off
    plot(mpc_time, mpc_controlEffort);
    hold on
    plot(bk_time, bk_controlEffort);
    title("Control Effort over Time");
    xlabel("time [s]");
    ylabel("Control Effort");
    legend("MPC", "Integral BK")
end