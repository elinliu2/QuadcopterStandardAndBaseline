function [ref, dist] = hardTrajectory()
    ref.waypointTracking = false;
    ref.x = @(t)                 6*cos(0.28*t)       + 1.8*cos(2.8*t)     + 0.6*sin(1.4*t);
    ref.x_dot = @(t)         -1.68*sin(0.28*t)      - 5.04*sin(2.8*t)    + 0.84*cos(1.4*t);
    ref.x_ddot = @(t)      -0.4704*cos(0.28*t)    - 14.112*cos(2.8*t)   - 1.176*sin(1.4*t);
    ref.x_dddot = @(t)    0.131712*sin(0.28*t)   + 39.5136*sin(2.8*t)  - 1.6464*cos(1.4*t);
    ref.x_ddddot = @(t) 0.03687936*cos(0.28*t) + 110.63808*cos(2.8*t) + 2.30496*sin(1.4*t);

    ref.y = @(t)              -2.25*sin(0.28*t)      - 0.3*sin(2.8*t)    - 0.45*cos(1.4*t);
    ref.y_dot = @(t)          -0.63*cos(0.28*t)     - 0.84*cos(2.8*t)    + 0.63*sin(1.4*t);
    ref.y_ddot = @(t)        0.1764*sin(0.28*t)    + 2.352*sin(2.8*t)   + 0.882*cos(1.4*t);
    ref.y_dddot = @(t)     0.049392*cos(0.28*t)   + 6.5856*cos(2.8*t)  - 1.2348*sin(1.4*t);
    ref.y_ddddot = @(t) -0.01382976*sin(0.28*t) - 18.43968*sin(2.8*t) - 1.72872*cos(1.4*t);

    ref.z = @(t) 2*sin(0.1*t)+10;
    ref.z_dot = @(t) 0.2*cos(0.1*t);
    ref.z_ddot = @(t) -0.02*sin(0.1*t);
    ref.z_dddot = @(t) -0.002*cos(0.1*t);
    ref.z_ddddot = @(t) 0.0002*sin(0.1*t);

    ref.psi = @(~) 0;
    ref.psi_dot = @(~) 0;
    ref.psi_ddot = @(~) 0;
    ref.psi_dddot = @(~) 0;
    ref.psi_ddddot = @(~) 0;

    dist.m = @(~) 0;
    dist.x = @(~) 0;
    dist.y = @(~) 0;
    dist.z = @(~) 0;

end