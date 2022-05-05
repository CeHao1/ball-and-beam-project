classdef studentControllerInterface < matlab.System
    properties (Access = private)
        %% You can add values that you want to store and updae while running your controller.
        % For more information of the supported data type, see
        % https://www.mathworks.com/help/simulink/ug/data-types-supported-by-simulink.html
        t_prev = -1;
        theta_d = 0;
        p_prev = 0;
        v_prev = 0;
        theta_prev = 0;
        theta_dot_prev = 0;

        V_servo_prev = 0;

        % MPC values
        dt = 0.02;
        H = 50; % horizon
        last_update_time = -1;

        % PID values
        integrated_position_error = 0;
        integrated_theta_error = 0;

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

%         if ((t*100-floor(t*100)) < 0.15)
%             fprintf("time %f \n", t);
%         end
           %% observer, calculate the x(2)=z_dot, x(4)=theta_dot
            t_prev = obj.t_prev;
            p_prev = obj.p_prev;
            theta_prev = obj.theta_prev;

            if(t_prev >= 0 && t > t_prev)
                dt = t - t_prev;
                v_ball = (p_ball - p_prev) /dt;
                theta_dot = (theta - theta_prev)/dt;
            elseif (t_prev == t)
                v_ball = obj.v_prev;
                theta_dot = obj.theta_dot_prev;
            else
                v_ball = 0;
                theta_dot = 0;
            end
            
            x = [p_ball, v_ball, theta, theta_dot]';
            %% Sample Controller: Simple Proportional Controller
            % Extract reference trajectory at the current timestep.
            [p_ball_ref, v_ball_ref, a_ball_ref] = get_ref_traj(t);

             
            %% judege the pattern of reference
            use_prediction = true;
            if (use_prediction)
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
    
                % predict reference in the horizon
                pred_ref = [];
                for i = 0:obj.H
                    t_pred = t + i*obj.dt;
                    [p_pred, v_pred, a_pred] = get_pred_traj(t_pred, wave_pattern, A, omega);
                    pred_ref = [pred_ref, [p_pred, v_pred]'];
                end
    
                ref = pred_ref;
            end
            %% controller

            % (1) default
%             [V_servo, theta_d] = default_controllor(x, ref);

            % (2) Feedback LQR
%             ref = [p_ball_ref, v_ball_ref]';
%             [V_servo, theta_d] = LQR_01(x, ref, obj.dt);

            % (3) MPC, we change change the control frequency
%             must use predicted trajectory
%             if ((t - obj.last_update_time)>0.005)
                [V_servo, theta_d] = linearized_MPC_01(x, ref, obj.H, obj.dt);
%                 obj.last_update_time = t;
%                 fprintf("use mpc");
%             else
%                 V_servo = obj.V_servo_prev;
%                 theta_d = obj.theta_prev;
%             end

            % (4) PID, two chains
%             predict = 1;
%             ref = [p_ball_ref, v_ball_ref]';
%             [V_servo, theta_d, obj.integrated_position_error, obj.integrated_theta_error] =  ...
%                 PID_01(x, ref, obj.dt, obj.integrated_position_error, obj.integrated_theta_error, predict);
            
            % Update class properties if necessary.
            obj.t_prev = t;
            obj.p_prev = p_ball;
            obj.v_prev = v_ball;
            obj.theta_prev = theta;
            obj.theta_dot_prev = theta_dot;
            obj.theta_d = theta_d;

            obj.V_servo_prev = V_servo;
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
