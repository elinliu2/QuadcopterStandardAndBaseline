classdef droneModel
    properties
        g;
        m;

        l; % l is the distance between Ob and each propeller; 
        K_psi; %  is the thrust-to-moment gain ;

        Jtheta;
        Jphi;
        Jpsi;

        numStates;
    end
    
    methods
        function obj = droneModel()
            obj.g = 9.8;
            obj.m = 1.4; % kg

            obj.l = 0.2; % m
            obj.K_psi = 4; % Nm

            obj.Jtheta = 0.03; % kgm^2
            obj.Jphi = 0.03; % kgm^2
            obj.Jpsi = 0.04; % kgm^2

            obj.numStates = 12;
        end

        function [time, stateProgression, controlEffort] = trajectory(model, SimConstants, initialState, showSim, showPlot, ref, dist, controller)
            % Drone Simulation
            if ref.waypointTracking
                i = 1;
                t = i*SimConstants.T;
                time = 0;
                stateProgression = zeros(12, 2);
                controlEffort = zeros(1, 2);
                stateProgression(:, 1) = initialState;
                while max(abs(stateProgression(:, i) - ref.state(t))) > ref.tol && t < SimConstants.t_final
                    i = i+1;
                    t = i*SimConstants.T;
                    time = [time t];
                    [dState, controller, control] = droneDynamics(i*SimConstants.T, stateProgression(:, i-1), model, ref, dist, controller);
                    stateProgression(:, i) = stateProgression(:, i-1) + dState*SimConstants.T;
                    controlEffort(:, i) = control;
                end
            else
                time = 0:SimConstants.T:SimConstants.t_final; 
                stateProgression = zeros(12, length(time));
                controlEffort = zeros(1, length(time));
                stateProgression(:, 1) = initialState;
                for i = 2:length(time)
                    [dState, controller, control] = droneDynamics(i*SimConstants.T, stateProgression(:, i-1), model, ref, dist, controller);
                    stateProgression(:, i) = stateProgression(:, i-1) + dState*SimConstants.T;
                    controlEffort(i) = control;
                end
            end

            
            % Plot
            if (showPlot)
                trajectoryPlot3d(time, stateProgression);
            end
            if (showSim)
                DisplaySim(time, stateProgression);
            end
            
        end

        function [dState, controller, controlEffort] = droneDynamics(t, state, model, ref, dist, controller)
            dx1 = state(4); 
            dx2 = state(5);
            dx3 = state(6);

            [F1, F2, F3, F4, controller] = controller.control(t, state, model, ref);
            controlEffort = norm(F1 + F2 + F3 + F4);

            theta = state(7); 
            phi = state(8); 
            psi = state(9); 

            dx4 = 1/(model.m + dist.m(t)) * (F1+F2+F3+F4) * (sin(theta)*cos(phi)*cos(psi) + sin(phi)*sin(psi)) + dist.x(t);
            dx5 = 1/(model.m + dist.m(t)) * (F1+F2+F3+F4) * (sin(theta)*cos(phi)*sin(psi) - sin(phi)*cos(psi)) + dist.y(t);
            dx6 = 1/(model.m + dist.m(t)) * (F1+F2+F3+F4) * (cos(theta)*cos(phi)) - model.g + dist.z(t);

            dx7 = state(10);
            dx8 = state(11);
            dx9 = state(12);

            dx10 = 1/(model.Jtheta) * model.l * (F1 - F2); 
            dx11 = 1/(model.Jphi) * model.l * (F3 - F4);
            dx12 = 1/(model.Jpsi) * model.K_psi * (F1 + F2 - F3 - F4);

            dState = [dx1; dx2; dx3; dx4; dx5; dx6; dx7; dx8; dx9; dx10; dx11; dx12];
        end
    end
end

function DisplaySim(time, stateProgression) 
    figure(2)
    sim = scatter3(0, 0, 0, 20, "blue", "filled");
    h_annot = annotation('textbox',[.9 .5 .1 .2],'String',0,'EdgeColor','none');
    title("Drone Sim");
    xlabel("x");
    ylabel("y");
    zlabel("z");
    zlim([0 15])

    for i = 1 : length(time)
        t = time(i);
        % update plot 
        sim.XData = stateProgression(1, i);
        sim.YData = stateProgression(2, i);
        sim.ZData = stateProgression(3, i);
        set(h_annot,'String',t)
        drawnow
        pause(0);
    end
end