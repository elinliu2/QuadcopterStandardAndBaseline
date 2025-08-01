function [ref, dist] = mediumTrajectory()
    ref.waypointTracking = false;
    ref.x = @(t) -1 + cos(0.5*t);
    ref.x_dot = @(t) -0.5*sin(0.5*t);
    ref.x_ddot = @(t) -0.25*cos(0.5*t);
    ref.x_dddot = @(t) 0.125*sin(0.5*t);
    ref.x_ddddot = @(t) 0.0625*cos(0.5*t);
    ref.y = @(t) sin(0.5.*t);
    ref.y_dot = @(t) 0.5*cos(0.5*t);
    ref.y_ddot = @(t) -0.25*sin(0.5*t);
    ref.y_dddot = @(t) -0.125*cos(0.5*t);
    ref.y_ddddot = @(t) 0.0625*sin(0.5*t);
    ref.z = @(~) 2;
    ref.z_dot = @(~) 0;
    ref.z_ddot = @(~) 0;
    ref.z_dddot = @(~) 0;
    ref.z_ddddot = @(~) 0;
    ref.psi = @(~) 0;
    ref.psi_dot = @(~) 0;
    ref.psi_ddot = @(~) 0;
    ref.psi_dddot = @(~) 0;
    ref.psi_ddddot = @(~) 0;

    dist.m = @(~) 0;
    dist.x = @(t) ( (1 <= t & t < 2)*0.525*sin(pi*(t-1)) ) + ...
               ( (3 <= t & t < 4)*1*sin(pi*(t-3)) ) + ...
               ( (5 <= t & t < 6)*5*sin(pi*(t-5)) );
    dist.y = @(t) ( (1 <= t & t < 2)*0.525*sin(pi*(t-1)) ) + ...
               ( (3 <= t & t < 4)*1*sin(pi*(t-3)) ) + ...
               ( (5 <= t & t < 6)*5*sin(pi*(t-5)) );
    dist.z = @(~) 0;

end