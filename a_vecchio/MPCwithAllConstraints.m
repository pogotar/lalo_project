% Constrained Model Predictive Control Function
% A: A matrix of the linear considered dynamic system
% B: B matrix of the linear considered dynamic system
% Q: status weight into the MPC cost function
% R: inputs weight into the MPC cost function
% S: final state weight (prediction horizon instant time) into the MPC cost function
% N: prediction horizon
% umin: inputs lower limit (scalar)
% umax: inputs upper limit (scalar)
% x2_min: state 2 lower limit (scalar)
% x2_max: state 2 upper limit (scalar)
% signX0_4: sign of the initial state x4 of the altitude
% over_shot_constraint: lower/upper bound for the altitude x4
% x: measured status at the current instant time
function u = MPCwithAllConstraints(A,B,Q,R,S,N,umin,umax,x2_min,x2_max,signX0_4,over_shot_constraint,slewrateMax,slewrateMin,x)
    %% Q and R matrices for Open-Loop MPC (with Q1)
    Qsig = blkdiag(kron(eye(N-1),Q),S);
    Rsig = kron(eye(N),R);

    %% A and B matrices for the Open-Loop MPC
    
    % A matrix
    Asig = A;
    for i = 2:N
        Asig = [Asig; A^i];
    end

    % B matrix
    Bsig = [];
    temp = [];
    for i = 1:N
        temp = zeros(size(B,1)*(i-1),size(B,2));
        for j = 0:N-i
            temp = [temp; A^(j)*B];
        end
        Bsig = [Bsig temp];
    end

    %% H, F 
    H = Bsig'*Qsig*Bsig + Rsig;
    H=(H+H')/2;
    F = Asig'*Qsig*Bsig;
    ft = x'*F;
    f=ft';
    
    % INPUTS Au*x<=bu
    %{
       u(k) <= umax
       -u(k) <= -umin --> u >= umin
       -u(k) + u(k+1) <= positive_slewrate
       u(k) - u(k+1) <= -negative_slewrate
    %}
    Au = [1 -1 -1 1]';
    Au1 = [0 0 1 -1]';
    bu = [umax -umin slewrateMax -slewrateMin]';
    Au_hat = [];
    for i = 1:N
        Au_hat_i = [];
        for j = 1:N
            if (i==j)
                Au_hat_i = [Au_hat_i Au];
            elseif (i==j-1)
                Au_hat_i = [Au_hat_i Au1];
            else
                Au_hat_i = [Au_hat_i [0 0 0 0]'];
            end
        end
        Au_hat = [Au_hat; Au_hat_i];
    end
    bu_hat = kron(ones(N,1),bu);
    
    % STATES Ax*x<=bx
    Ax = [0 1 0 0; 0 -1 0 0; 0 0 0 -signX0_4];
    bx = [x2_max -x2_min -signX0_4*over_shot_constraint]';
    Ax_hat = kron(eye(N),Ax);
    bx_hat = kron(ones(N,1), bx);
    
    % INPUT AND STATES
    A_hat = [Au_hat; (Ax_hat*Bsig)];
    b_hat = [bu_hat; (bx_hat-Ax_hat*Asig*x)];
    
    options = optimset('Algorithm','interior-point-convex',...
        'Diagnostics','off','LargeScale','off','Display','off');
    %solve the quadratic programming problem
    [U,fval,exitflag,output] = quadprog(H,f,A_hat,b_hat,[],[],[],[],[],options);
    
    if isempty(U)
        [U,fval,exitflag,output] = quadprog(H,f,Au_hat,bu_hat,[],[],[],[],[],options);
    end

    %get the optimal input value (the receding horizon principle is applied)
    u = U(1);
end