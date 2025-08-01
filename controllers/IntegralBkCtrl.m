classdef IntegralBkCtrl
    properties
        x; % Internal Controller State
        % x_dot(1) = theta_ref - theta
        % x_dot(2) = phi_ref - phi
        % x_dot(3) = psi_ref - psi
        % x_dot(4) = z_ref - z
        % x_dot(5) = x_ref - x
        % x_dot(6) = y_ref - y
        % x_dot(7) = U_z dot 
        % x_dot(8) = theta_ref dot
        % x_dot(9) = phi_ref dot
        % x_dot(10) = theta_ref double dot
        % x_dot(11) = phi_ref double dot

        % Controller parameters
        c1;
        c2;
        c3;
        c4;
        c5;
        c6;
        c7;
        c8;
        c9;
        c10;
        c11;
        c12;

        lambda1;
        lambda2;
        lambda3;
        lambda4;
        lambda5;
        lambda6;

        dT;

        prev_theta_ref;
        prev_phi_ref;
        prev_theta_dot_ref;
        prev_phi_dot_ref;
    end
    
    methods
        function obj = IntegralBkCtrl(timestep, droneState, drone, ref)
            obj.c1 = 10.5;
            obj.c2 = 2;
            obj.c3 = 10;
            obj.c4 = 2;
            obj.c5 = 2;
            obj.c6 = 2;
            obj.c7 = 4;
            obj.c8 = 2;
            obj.c9 = 1;
            obj.c10 = 0.5;
            obj.c11 = 1;
            obj.c12 = 0.5;
            obj.lambda1 = 1e-1;
            obj.lambda2 = 1e-1;
            obj.lambda3 = 1e-1;
            obj.lambda4 = 1e-1;
            obj.lambda5 = 1e-1;
            obj.lambda6 = 1e-1;

            obj.dT = timestep;

            obj.prev_theta_ref = 0;
            obj.prev_phi_ref = 0;
            obj.prev_theta_dot_ref = 0;
            obj.prev_phi_dot_ref = 0;

            obj.x = zeros(12, 1);
            
            obj.x(7) = U_z(obj, 0, droneState, drone, ref);
        end
        
        function [F1, F2, F3, F4, ctrl] = control(ctrl, t, droneState, drone, ref)
            drone1 = droneState(1);
            drone2 = droneState(2);
            drone3 = droneState(3);
            drone6 = droneState(6);
            drone7 = droneState(7);
            drone8 = droneState(8);
            drone9 = droneState(9);
            drone10 = droneState(10);
            drone11 = droneState(11);
            drone12 = droneState(12);

            ctrl1 = ctrl.x(1);
            ctrl2 = ctrl.x(2);
            ctrl3 = ctrl.x(3);
            ctrl4 = ctrl.x(4);
            ctrl7 = ctrl.x(7);
            ctrl8 = ctrl.x(8);
            ctrl9 = ctrl.x(9);
            ctrl10 = ctrl.x(10);
            ctrl11 = ctrl.x(11);

            e7 = ref.z(t) - drone3;
            e8 = ctrl.c7*e7 + ref.z_dot(t) + ctrl.lambda4*ctrl4 - drone6;
            U_z = ctrl7;
        
            theta_reference = theta_ref(ctrl, t, droneState, drone, ref);
            phi_reference =   phi_ref(ctrl, t, droneState, drone, ref);
            
            theta_ref_dot = ctrl8;
            phi_ref_dot = ctrl9;
            theta_ref_ddot = ctrl10;
            phi_ref_ddot = ctrl11;
        
            e1 = theta_reference - drone7;
            e2 = ctrl.c1*e1 + theta_ref_dot + ctrl.lambda1*ctrl1 - drone10;
            e3 = phi_reference - drone8;
            e4 = ctrl.c3*e3 + phi_ref_dot + ctrl.lambda2*ctrl2 - drone11;
            e5 = ref.psi(t) - drone9;
            e6 = ctrl.c5*e5 + ref.psi_dot(t) + ctrl.lambda3*ctrl3 - drone12;
        
            z_ddot = 4*U_z*cos(drone7)*cos(drone8)/drone.m-drone.g;
            e8_dot = ctrl.c7*(ref.z_dot(t) - drone6)+ref.z_ddot(t)+ctrl.lambda4*e7-z_ddot;
            ez     = (1-ctrl.c7^2+ctrl.lambda4)*e7+(ctrl.c7+ctrl.c8)*e8-ctrl.c7*ctrl.lambda4*ctrl4+drone.g;
            ez_dot = ((1-ctrl.c7^2+ctrl.lambda4)*(ref.z_dot(t)-drone6)+(ctrl.c7+ctrl.c8)*e8_dot-ctrl.c7*ctrl.lambda4*e7);
            U_zdot = drone.m/4*( cos(drone7)*cos(drone8)*ez_dot + ez*(-sin(drone7)*cos(drone8)*drone10-cos(drone7)*sin(drone8)*drone11) )/(cos(drone7)*cos(drone8))^2;
        
            U_theta = drone.Jtheta/(2*drone.l)*((1-ctrl.c1^2+ctrl.lambda1)*e1 + (ctrl.c1+ctrl.c2)*e2 + theta_ref_ddot - ctrl.c1*ctrl.lambda1*ctrl1);
            U_phi = drone.Jphi/(2*drone.l)*((1-ctrl.c3^2+ctrl.lambda2)*e3 + (ctrl.c3+ctrl.c4)*e4 + phi_ref_ddot - ctrl.c3*ctrl.lambda2*ctrl2);
            U_psi = drone.Jpsi/(4*drone.K_psi)*((1-ctrl.c5^2+ctrl.lambda3)*e5 + (ctrl.c5+ctrl.c6)*e6 + ref.psi_ddot(t) - ctrl.c5*ctrl.lambda3*ctrl3);
        
            F1 = U_theta + U_psi + U_z;
            F2 = -U_theta + U_psi + U_z;
            F3 = U_phi - U_psi + U_z;
            F4 = -U_phi - U_psi + U_z;

            % Update internal controller state
            ctrl.x(1) = ctrl.x(1) + (theta_reference - drone7)*ctrl.dT;
            ctrl.x(2) = ctrl.x(2) + (phi_reference - drone8)*ctrl.dT;
            ctrl.x(3) = ctrl.x(3) + (ref.psi(t) - drone9)*ctrl.dT;
            ctrl.x(4) = ctrl.x(4) + (ref.z(t) - drone3)*ctrl.dT;
            ctrl.x(5) = ctrl.x(5) + (ref.x(t) - drone1)*ctrl.dT;
            ctrl.x(6) = ctrl.x(6) + (ref.y(t) - drone2)*ctrl.dT;
        
            ctrl.x(7) = ctrl.x(7) + (U_zdot)*ctrl.dT;

            ctrl.x(8) = 1/ctrl.dT*(theta_reference - ctrl.prev_theta_ref );
            ctrl.x(9) = 1/ctrl.dT*(phi_reference - ctrl.prev_phi_ref);
            ctrl.x(10) = 1/ctrl.dT*(theta_ref_dot - ctrl.prev_theta_dot_ref);
            ctrl.x(11) = 1/ctrl.dT*(phi_ref_dot - ctrl.prev_phi_dot_ref);

            ctrl.prev_theta_ref = theta_reference;
            ctrl.prev_phi_ref = phi_reference;
            ctrl.prev_theta_dot_ref = theta_ref_dot;
            ctrl.prev_phi_dot_ref = phi_ref_dot;
        end
    end
end

function U_z = U_z(ctrl, t, droneState, drone, ref)
    drone3 = droneState(3);
    drone6 = droneState(6);
    drone7 = droneState(7);
    drone8 = droneState(8);
    ctrl4 = ctrl.x(4);
    
    e7 = ref.z(t) - drone3;
    e8 = ctrl.c7*e7 + ref.z_dot(t) + ctrl.lambda4*ctrl4 - drone6;
    U_z = drone.m/(cos(drone7)*cos(drone8)*4)*(drone.g + (1-ctrl.c7^2+ctrl.lambda4)*e7 + (ctrl.c7+ctrl.c8)*e8 - ctrl.c7*ctrl.lambda4*ctrl4);
end 

function theta_ref = theta_ref(ctrl, t, droneState, drone, ref)
    drone1 = droneState(1);
    drone4 = droneState(4);
    ctrl5 = ctrl.x(5);
    ctrl7 = ctrl.x(7);

    e9 = ref.x(t) - drone1;
    e10 = ctrl.c9*e9 + ref.x_dot(t) + ctrl.lambda5*ctrl5 - drone4;
    U_z = ctrl7;

    theta_ref = drone.m/U_z*((1-ctrl.c9^2  + ctrl.lambda5)*e9 + (ctrl.c9 + ctrl.c10)*e10 + ref.x_ddot(t) - ctrl.c9*ctrl.lambda5*ctrl5);
end

function phi_ref = phi_ref(ctrl, t, droneState, drone, ref)
    drone2 = droneState(2);
    drone5 = droneState(5);
    ctrl6 = ctrl.x(6);
    ctrl7 = ctrl.x(7);

    e11 = ref.y(t) - drone2;
    e12 = ctrl.c11*e11 + ref.y_dot(t) + ctrl.lambda6*ctrl6 - drone5;
    U_z = ctrl7;

    phi_ref =   drone.m/U_z*( -(1-ctrl.c11^2 + ctrl.lambda6)*e11 - (ctrl.c11+ctrl.c12)*e12 - ref.y_ddot(t) + ctrl.c11*ctrl.lambda6*ctrl6);
end
