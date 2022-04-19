function [V_servo, theta_d] = LQR_01(x, ref, dt)
    if(~exist('dt, var'))
        dt = 0.01;
    end

    w_p= 1; %20 level
    w_v= 1; % 20
    w_theta = 1;
    Q = diag([w_p, w_v, w_theta, 0]);
    R = [1];

    [A, B] = jacobian_linearization(x);
    x0 = x - [ref(1,1); 0; 0; 0]; % e = z - zref
       
    K = lqr(A, B, Q, R);
    V_servo = -K*x0;

    x_dot = (A - B*K)*x0;
    theta_d = x(3) + x_dot(3) * dt;

end