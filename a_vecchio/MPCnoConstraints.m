% Constrained Model Predictive Control Function
% A: A matrix of the linear considered dynamic system
% B: B matrix of the linear considered dynamic system
% Q: status weight into the MPC cost function
% R: inputs weight into the MPC cost function
% S: final state weight (prediction horizon instant time) into the MPC cost function
% N: prediction horizon
% x: measured status at the current instant time
function u = MPCnoConstraints(A,B,Q,R,S,N,x)
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

    options = optimset('LargeScale','off','Display','off');
    %solve the quadratic programming problem
    [U,fval,exitflag,output] = quadprog(H,f,[],[],[],[],[],[],[],options);

    %get the optimal input value (the receding horizon principle is applied)
    u = U(1);
end