function [ref, dist] = easyTrajectory(x, y, m_dist)
    ref.waypointTracking = true;
    ref.tol = 0.05;
    ref.x = @(t) x;
    ref.x_dot = @(~) 0;
    ref.x_ddot = @(~) 0;
    ref.x_dddot = @(~) 0;
    ref.x_ddddot = @(~) 0;
    ref.y = @(t) y;
    ref.y_dot = @(~) 0;
    ref.y_ddot = @(~) 0;
    ref.y_dddot = @(~) 0;
    ref.y_ddddot = @(~) 0;
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
    ref.state = @(t) [ref.x(t); ref.y(t); ref.z(t); 
                    ref.x_dot(t); ref.y_dot(t); ref.z_dot(t);
                    0; 0; 0; 0; 0; 0;];

    dist.m = @(~) m_dist;
    dist.x = @(~) 0;
    dist.y = @(~) 0;
    dist.z = @(~) 0;

end