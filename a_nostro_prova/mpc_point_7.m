% Constrained Model Predictive Control Function
% A: A matrix of the linear considered dynamic system
% B: B matrix of the linear considered dynamic system
% Q: status weight into the MPC cost function
% R: inputs weight into the MPC cost function
% S: final state weight (prediction horizon instant time) into the MPC cost function
% N: prediction horizon
% umin: inputs lower limit (scalar)
% umax: inputs upper limit (scalar)
% X: measured status at the current instant time
% x initial state of every iteration        

function u = mpc_point_7(A,B,Q,R,S,N,u_min,u_max,x2_min,x2_max,over_shot,start_sign,slewratemin,slewratemax,x)
    m = size(B,2);    % number of inputs
    n = size(A,1);    % number of states
    
    %% Q and R matrices for Open-Loop MPC (with Q1)
    Qsig = blkdiag( kron(eye(N-1),Q) ,S);
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
    % M not necessary in theoptimisation
    H = Bsig'*Qsig*Bsig + Rsig;
    H=(H+H')/2;
    F = Asig'*Qsig*Bsig;
    ft = x'*F;
    f=ft';

    % input and status constraints definition
    lb = [repmat(u_min, N*m,1)];  % lower bound of x
    ub = [repmat(u_max, N*m,1)];  % upper bound of x
    % refmat repeats a matrix several times

    % STATES
    Ax = [0 1  0 0
          0 -1 0 0
          0 0  0 -start_sign];
    bx = [ x2_max
          -x2_min
          -start_sign*over_shot];


    Ax_hat = [kron(eye(N),Ax)];
    bx_hat = [kron(ones(N,1), bx)];

    bu = [slewratemax -slewratemin]';
    Au = [-1 1]';
    Au1 = [1 -1]';
    Au_hat = [];
    for i = 1:N
        Au_hat_i = [];
        for j = 1:N
            if (i==j)
                Au_hat_i = [Au_hat_i Au];
            elseif (i==j-1)
                Au_hat_i = [Au_hat_i Au1];
            else
                Au_hat_i = [Au_hat_i [0 0]'];
            end
        end
        Au_hat = [Au_hat; Au_hat_i];
    end
    bu_hat = kron(ones(N,1),bu);

    % INPUT AND STATES
    A_hat = [Ax_hat*Bsig
             Au_hat         ];
    b_hat = [bx_hat-Ax_hat*Asig*x
             bu_hat               ];

    options = optimset('Algorithm', 'interior-point-convex','Diagnostics','off', ...
        'Display','off'); % to toggle off some info that quadprog returns
    %solve the quadratic programming problem
    U = quadprog(H,f,A_hat,b_hat,[],[],lb,ub,[],options);

    if isempty(U)
        disp('too restrictive contraints')
    end


    %get the optimal input value (the receding horizon principle is applied)
    u = U(1:m); % apply only first one of the opt control sequence
    % receiding horizon principle

end
    
