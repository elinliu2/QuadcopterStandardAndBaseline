addpath(genpath('.'))
SimConstants.T = 0.01;
SimConstants.t_final = 20;
SimConstants.options = odeset('RelTol',1e-12, 'abstol', 1e-12); 

drone = droneModel();
initialState = zeros(drone.numStates, 1);

tic;
%% FOR EASY TRAJECTORY
% waypoints = [0 0; 0 1; 1 1; 1 0];
% m_dist = [drone.m*0.0 drone.m*0.0 0 0];
% total_time = 0;
% total_stateProgression = zeros(12, 1); 
% total_controlEffort = 0;
% rmse_error = 0;
% mte = 0;
% for i = 1:4
%     [ref, dist] = easyTrajectory(waypoints(i, 1), waypoints(i, 2), m_dist(i));
%
%     ctrller = IntegralBkCtrl(SimConstants.T, initialState, drone, ref);
%     ctrller = MpcCtrl(SimConstants.T, drone);
%
%     [time, stateProgression, controlEffort] = trajectory(drone, SimConstants, initialState, false, false, ref, dist, ctrller);
%     initialState = stateProgression(:, length(time));
%     total_time = [total_time time+total_time(length(total_time))];
%     total_stateProgression = [total_stateProgression stateProgression];
%     total_controlEffort = [total_controlEffort controlEffort];
%     ref_vec = [waypoints(i, 1); waypoints(i, 2); 2]*ones(1, length(time));
%     rmse_error = [rmse_error rmse(ref_vec, stateProgression(1:3, :), 1)];
%     mte = max(mte, max(vecnorm(ref_vec-stateProgression(1:3, :))));
%  end

%% FOR MEDIUM AND HARD TRAJECTORY
[ref, dist] = mediumTrajectory();
% [ref, dist] = hardTrajectory();

ctrller = IntegralBkCtrl(SimConstants.T, initialState, drone, ref);
% ctrller = MpcCtrl(SimConstants.T, drone);

[total_time, total_stateProgression, total_controlEffort] = trajectory(drone, SimConstants, initialState, false, false, ref, dist, ctrller);
ref_vec = [arrayfun(ref.x, total_time); arrayfun(ref.y, total_time); arrayfun(ref.z, total_time);
    arrayfun(ref.x_dot, total_time); arrayfun(ref.y_dot, total_time);arrayfun(ref.y_dot, total_time)];
rmse_error = rmse(ref_vec, total_stateProgression(1:6, :), 1);
mte = max(vecnorm(ref_vec-total_stateProgression(1:6, :)));
toc;

%% FOR PLOTTING
trajectoryPlot3d(total_time, total_stateProgression);
errorPlot(total_time, rmse_error);
controlEffortPlot(total_time, total_controlEffort);
disp("MTE:" + string(mte))