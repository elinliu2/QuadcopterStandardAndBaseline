classdef MpcCtrl
    properties
       ctrl;
       prevU;
    end
    
    methods
        function obj = MpcCtrl(timestep, drone)
            obj.ctrl = nlmpc(12,12,4);
            obj.ctrl.Ts = timestep;
            obj.ctrl.PredictionHorizon = 20;
            obj.ctrl.ControlHorizon = 5;
            obj.ctrl.Model.IsContinuousTime = false;
            obj.ctrl.Weights.OutputVariables = [100 100 500 100 100 100 1 1 1 1 1 1];
            obj.ctrl.MV = struct( ...
                Min={0;0;0;0}, ...
                RateMin={-2;-2;-2;-2}, ...
                RateMax={2;2;2;2} ...
                );
            obj.ctrl.Weights.ManipulatedVariables = [0.5 0.5 0.5 0.5];
            obj.ctrl.Weights.ManipulatedVariablesRate = [0.5 0.5 0.5 0.5];
            obj.prevU = drone.m * drone.g / 4 * ones(4, 1);
        end
        
        function [F1, F2, F3, F4, obj] = control(obj, t, x, drone, ref)
            obj.ctrl.Model.StateFcn = @(x,u) droneStateFunction(x, u, obj.ctrl.Ts, drone);
            ref = [ref.x(t) ref.y(t) ref.z(t) ref.x_dot(t) ref.y_dot(t) ref.z_dot(t) 0 0 0 0 0 0];
            
            
            nlopts = nlmpcmoveopt;
            nlopts.MVTarget = drone.m * drone.g / 4 * ones(4, 1); 

            u = nlmpcmove(obj.ctrl, x, obj.prevU, ref); 
            
            obj.prevU = u;
            F1 = u(1);
            F2 = u(2);
            F3 = u(3);
            F4 = u(4);
        end
    end
end

function xk1 = droneStateFunction(xk, uk, Ts, drone)
    dx1 = xk(4); 
    dx2 = xk(5);
    dx3 = xk(6);

    theta = xk(7); 
    phi = xk(8); 
    psi = xk(9); 

    dx4 = 1/(drone.m) * (uk(1)+uk(2)+uk(3)+uk(4)) * (sin(theta)*cos(phi)*cos(psi) + sin(phi)*sin(psi));
    dx5 = 1/(drone.m) * (uk(1)+uk(2)+uk(3)+uk(4)) * (sin(theta)*cos(phi)*sin(psi) - sin(phi)*cos(psi));
    dx6 = 1/(drone.m) * (uk(1)+uk(2)+uk(3)+uk(4)) * (cos(theta)*cos(phi)) - drone.g;

    dx7 = xk(10);
    dx8 = xk(11);
    dx9 = xk(12);

    dx10 = 1/(drone.Jtheta) * drone.l * (uk(1)-uk(2)); 
    dx11 = 1/(drone.Jphi) * drone.l * (uk(3)-uk(4));
    dx12 = 1/(drone.Jpsi) * drone.K_psi * (uk(1)+uk(2)-uk(3)-uk(4));

    dState = [dx1; dx2; dx3; dx4; dx5; dx6; dx7; dx8; dx9; dx10; dx11; dx12];

    xk1 = xk + dState*Ts;
end
