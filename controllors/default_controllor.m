function [V_servo, theta_d] = default_controllor(x, ref)

p_ball = x(1);
theta = x(3);
p_ball_ref = ref(1,1);

% Decide desired servo angle based on simple proportional feedback.
k_p = 3;
theta_d = - k_p * (p_ball - p_ball_ref);

% Make sure that the desired servo angle does not exceed the physical
% limit. This part of code is not necessary but highly recommended
% because it addresses the actual physical limit of the servo motor.
theta_saturation = 56 * pi / 180;    
theta_d = min(theta_d, theta_saturation);
theta_d = max(theta_d, -theta_saturation);

% Simple position control to control servo angle to the desired
% position.
k_servo = 10;
V_servo = k_servo * (theta_d - theta);


end