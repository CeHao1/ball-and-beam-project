function [V_servo, theta_d] = LQR_01(x, ref, dt)
%     if(~exist('dt, var'))
%         dt = 0.01;
%     end

    w_p= 20; 
    w_v= 0; 
    w_theta = 0;
    Q = diag([w_p, w_v, w_theta, 0]);
    R = [1];

    [A, B] = jacobian_linearization(x);
    x0 = x - [ref(1,1); 0; 0; 0]; % e = z - zref
       
    coder.extrinsic('lqr');
    [K_raw, S, e] = lqr(A, B, Q, R);
%     [X, K_raw, L] = icare(A, B, Q, R);

    K = zeros(1,4);
    K = K_raw;

    V_servo = -K*x0;

    x_dot = (A - B*K)*x0;
    theta_d = x(3) + x_dot(3) * dt;

end