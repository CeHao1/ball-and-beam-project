classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        p_prev = 0;
        theta_prev = 0;

        % dt
        dt = 0.01;
        H = 20; % horizon

        % wave pattern, 0-square, 1-sine
        wave_pattern = 0; 
        A = 0;
        omega = 0;
    end
    methods(Access = protected)
        % function setupImpl(obj)
        %    disp("You can use this function for initializaition.");
        % end

        function V_servo = stepImpl(obj, t, p_ball, theta)
        % This is the main function called every iteration. You have to implement
        % the controller in this function, bu you are not allowed to
        % change the signature of this function. 
        % Input arguments:
        %   t: current time
        %   p_ball: position of the ball provided by the ball position sensor (m)
        %
        %   theta: servo motor angle provided by the encoder of the motor (rad)
        % Output:
        %   V_servo: voltage to the servo input.
           %% observer, calculate the x(2)=z_dot, x(4)=theta_dot
            t_prev = obj.t_prev;
            p_prev = obj.p_prev;
            theta_prev = obj.theta_prev;

            if(t_prev > 0)
                dt = t - t_prev;
                v_ball = (p_ball - p_prev) /dt;
                theta_dot = (theta - theta_prev)/dt;
            else
                v_ball = 0;
                theta_dot = 0;
            end
            

            %% Sample Controller: Simple Proportional Controller
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

             %% judege the pattern of reference
             if (a_ball_ref == 0) 
                 if (obj.wave_pattern == 1) % if the last pattern is sine wave
                    wave_pattern = obj.wave_pattern;
                    A = obj.A;
                    omega = obj.omega;
                 else % use square wave instead
                    wave_pattern = 0; % square
                    A = p_ball_ref;
                    omega = 0;
                 end
             else
                 wave_pattern = 1;
                 omega = sqrt(-a_ball_ref / p_ball_ref);
                 A = p_ball_ref / sin(omega * t);
             end

             obj.wave_pattern = wave_pattern;
             obj.A = A;
             obj.omega = omega;

%              fprintf("a %.3f, p %.3f ", a_ball_ref, a_ball_ref);
%              fprintf("w %d, A %.3f, o %.3f \n", wave_pattern, A, omega);

            % predict reference in the horizon
            pred_ref = [];
            for i = 0:obj.H
                t_pred = t + i*obj.dt;
                [p_pred, v_pred, a_pred] = get_pred_traj(t_pred, wave_pattern, A, omega);
                pred_ref = [pred_ref, [p_pred, v_pred]'];
%                 fprintf("%.3f,  ", pred_ref(end));
            end
%             fprintf("\n ");

            x = [p_ball, v_ball, theta, theta_dot]';
            ref = pred_ref;

            %% controller
%             [V_servo, theta_d] = default_controllor(x, ref);
            [V_servo, theta_d] = LQR_01(x, ref);
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.theta_d = theta_d;
        end
    end
    
    methods(Access = public)
        % Used this for matlab simulation script. fill free to modify it as
        % however you want.
        function [V_servo, theta_d] = stepController(obj, t, p_ball, theta)        
            V_servo = stepImpl(obj, t, p_ball, theta);
            theta_d = obj.theta_d;
        end
    end
    
end
