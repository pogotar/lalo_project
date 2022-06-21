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
function u = mympc(A,B,Q,R,S,N,umin,umax,x)
    
    %% Compute calligraphic Q and R matrices 

    %% Compute calligraphic A and B matrices 
   

    %% Compute H, F and derive f

    %% Define input and status constraints 
    
    %% Solve the opt problem using quadprog
    options = optimset('Algorithm', 'interior-point-convex','Diagnostics','off', ...
        'Display','off');
    U = quadprog(H,f,[],[],[],[],lb,ub,[],options);

    %% Apply receding horizon principle
end