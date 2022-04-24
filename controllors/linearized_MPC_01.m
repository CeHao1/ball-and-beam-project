function [V_servo, theta_d] = linearized_MPC_01(x, ref, Np, dt)
    w_p= 20; 
    w_v= 0; 
    w_theta = 0;
    Q = [w_p, w_v, w_theta, 0];
    R = [1];

    p_max = 0.19;
    theta_max = 60/180*pi;

    Ns = 4;
    Nc = 1;
    Nv_s = Ns*(Np+1);
    Nv_c = Nc * Np;
    Nv = Nv_s + Nv_c;
    I = eye(Ns);

    x0 = x;

    [A, B] = jacobian_linearization(x);

    Ad = I + A*dt;
    Bd = B*dt;
    %% formulate H, f
    H = diag([repmat(Q, 1, Np+1), repmat(R, 1, Np)]);
    x_ref = zeros(Nv, 1);
    x_ref(1:Ns:Nv_s) = ref(1,:)'; % ref_pos
    x_ref(2:Ns:Nv_s) = ref(2,:)'; % ref_vel
    
    f = -1 * x_ref' * H;

    %% formulate equality
    A_combined1 = repmat({zeros(Ns,Ns)}, Np+1, Np+1);
    A_combined2 = repmat({zeros(Ns,Nc)}, Np+1, Np);
    D_combined = repmat({zeros(Ns,1)}, Np+1, 1);

    A_combined1{1,1} = I;
    D_combined{1,1} = x0;
    for i=2:Np+1
        A_combined1{i,i} = I;
        A_combined1{i,i-1} = -Ad;
        A_combined2{i,i-1} = -Bd;
    end

    A_combined = [A_combined1, A_combined2];
    A_qp = cell2mat(A_combined);
    D_qp = cell2mat(D_combined);

    %% formulate inequality
    

    %% solve
%     [x_solved, existflag] = mpcActiveSetSolver(H, f, [], [], A_qp, D_qp);

%     options = optimoptions('quadprog', 'Display', 'iter');
    options = optimset('display', 'off');
    [xout, fval, exitflag] = quadprog(H,f, [],[], A_qp, D_qp, [],[],[],options);

%     cvx_begin
%         variable xout(Nv);
%         minimize(xout' * H * xout + 2*f*xout)
%         subject to
%             A_qp*xout == D_qp
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

%     figure
%     subplot(1,2,1)
%     plot(x_solved(1,:));
%     title('pos')
%     subplot(1,2,2)
%     plot(u_solved);
%     title('V servo')

    V_servo = u_solved(1);
    theta_d = x_solved(3,2);

end