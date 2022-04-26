function [V_servo, theta_d, I_p, I_theta] = PID_01(x, ref, dt, I_p, I_theta, predict)

%% from ball position, velocity to desired theta
Kp_p = 13;
Ki_p = 0.01;
Kd_p = 10;

p_error = ref(1, predict) - x(1);
v_error = ref(2, predict) - x(2);

I_p = I_p + p_error * dt;
theta_d = Kp_p * p_error + Kd_p * v_error + Ki_p * I_p;

% if the position is very close to the edge
% we can also modify the desired theta

angle = 20; % use the limit angle to smooth the actions
% theta_saturation = 56 * pi / 180;    
theta_saturation = angle * pi / 180;  
theta_d = min(theta_d, theta_saturation);
theta_d = max(theta_d, -theta_saturation);

%% from theta to servo
Kp_s = 1;
Ki_s = 0.05;
Kd_s = 0.05;

theta_error = theta_d - x(3);
theta_dot_error = 0 - x(4);

I_theta = I_theta + theta_error * dt;
V_servo = Kp_s * theta_error + Kd_s * theta_dot_error + I_theta * Ki_s;


end