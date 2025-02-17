function [V_servo, theta_d] = linearized_MPC_01(x, ref, Np, dt)
    w_p= 20; 
    w_v= 1e-5; 
    w_theta = 1e-5;
    w_theta_dot = 1e-5;
    Q = [w_p, w_v, w_theta, w_theta_dot];
    R = [1];

    w_p_terminal = 100;
    Q_terminal = [w_p_terminal, w_v, w_theta, w_theta_dot];

    p_max = 0.191;
    theta_max = 60/180*pi;

    Ns = 4;
    Nc = 1;
    Nv_s = Ns*(Np+1);
    Nv_c = Nc * Np;
    Nv = Nv_s + Nv_c;
    I = eye(Ns);

    x0 = x;

    [A, B] = jacobian_linearization(x);
%     [A, B] = integrator(4);

    Ad = I + A*dt;
    Bd = B*dt;
    %% formulate H, f
    H = diag([repmat(Q, 1, Np),Q_terminal,  repmat(R, 1, Np)]);
    x_ref = zeros(Nv, 1);
    x_ref(1:Ns:Nv_s) = ref(1,:)'; % ref_pos
    x_ref(2:Ns:Nv_s) = ref(2,:)'; % ref_vel
    
    f = -1 * x_ref' * H;

    x_init = x_ref;

    %% new A, D
    A_combined1 = zeros(Nv_s, Nv_s);
    A_combined2 = zeros(Nv_s, Nv_c);
    D_combined = zeros(Nv_s, 1);

    A_combined1(1:Ns, 1:Ns) = I;
    D_combined(1:Ns, 1) = x0;
    ToNs = 1:Ns;
    ToNc = 1:Nc;
    for i=2:Np+1
        A_combined1((i-1)*Ns+ ToNs, (i-1)*Ns+ ToNs)=I;
        A_combined1((i-1)*Ns+ ToNs, (i-2)*Ns+ ToNs)= -Ad;
        A_combined2((i-1)*Ns+ ToNs, (i-2)*Nc+ ToNc)= -Bd;
    end
    A_qp = [A_combined1, A_combined2];
    D_qp = D_combined;
    %% formulate equality
%     A_combined1 = repmat({zeros(Ns,Ns)}, Np+1, Np+1);
%     A_combined2 = repmat({zeros(Ns,Nc)}, Np+1, Np);
%     D_combined = repmat({zeros(Ns,1)}, Np+1, 1);
% 
%     A_combined1{1,1} = I;
%     D_combined{1,1} = x0;
%     for i=2:Np+1
%         A_combined1{i,i} = I;
%         A_combined1{i,i-1} = -Ad;
%         A_combined2{i,i-1} = -Bd;
%     end

%     A_combined = [A_combined1, A_combined2];
%     A_qp = cell2mat(A_combined);

%     coder.extrinsic('cell2mat');
%     A_qp1 = cell2mat(A_combined1);
%     A_qp2 = cell2mat(A_combined2);
%     A_qp = [A_qp1, A_qp2];
%     D_qp = cell2mat(D_combined);

    %% formulate inequality
    x_up = 1000*ones(Nv, 1);
    x_dn = -1000*ones(Nv, 1);

    % x
    x_up(1:Ns:Nv_s) = p_max;
    x_dn(1:Ns:Nv_s) = -p_max;
    
%     A_inequ1 = zeros(Np, Nv);
%     for i = 1:Np
%         A_inequ1(i, 1+i*Ns) = 1;
%     end
%     b_inequ1 = ones(Np, 1) * p_max;
% 
%     A_inequ1 = [A_inequ1; -A_inequ1];
%     b_inequ1 = [b_inequ1; b_inequ1];

    % theta
    x_up(3:Ns:Nv_s) = theta_max;
    x_dn(3:Ns:Nv_s) = -theta_max;

%     A_inequ2 = zeros(Np, Nv);
%     for i = 1:Np
%         A_inequ2(i, 3+i*Ns) = 1;
%     end
%     b_inequ2 = ones(Np, 1) * theta_max;
% 
%     A_inequ2 = [A_inequ2; -A_inequ2];
%     b_inequ2 = [b_inequ2; b_inequ2];
% 
%     A_inequ = [A_inequ1; A_inequ2];
%     b_inequ = [b_inequ1; b_inequ2];


    %% solve
%     f = f';
%     iA0 = false(size(b_inequ));

%     options = mpcActiveSetOptions;
%     options.MaxIterations = 10;
%     opt.ConstraintTolerance = 1.0e-4;
%     [xout, existflag] = mpcActiveSetSolver(H, f, A_inequ, b_inequ, A_qp, D_qp, iA0, options);



   %% solver quadprog
%     coder.extrinsic('quadprog');

    options = optimoptions('quadprog', 'Algorithm', 'active-set');
%     options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex');
    options.Display = 'off';
    
%     options.MaxIterations = 11;
%     options.StepTolerance = 1e-4;
%     [xout_raw, fval, exitflag] = quadprog(H,f, A_inequ, b_inequ, A_qp, D_qp, [],[],[],options);
%     tic
    [xout, fval, exitflag] = quadprog(H,f, [],[], A_qp, D_qp, x_dn,x_up, x_init, options);
%     toc
% 
% options = optimoptions('quadprog', 'Display', 'iter');

   %% solver cvx
%     cvx_begin
%         variable xout(Nv);
%         minimize(xout' * H * xout + f*xout)
%         subject to
%             A_qp*xout == D_qp   
%             A_inequ*xout <= b_inequ;
% %             x_dn <= xout <= x_up
%     cvx_end

    %% get results
    x_solved = zeros(Ns, Np+1);
    u_solved = zeros(Nc, Np);

    for i = 1:Ns
        x_solved(i,:) = xout(i:Ns:Nv_s)';
    end

    for i =1:Nc
        u_solved(i,:) = xout(Nv_s + (i:Nc:Nv_c))';
    end

%     MPC_plot(x_solved, u_solved);

    V_servo = u_solved(1);
    theta_d = x_solved(3,2);

end