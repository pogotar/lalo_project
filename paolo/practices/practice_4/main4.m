% MPC

clear all; close all; clc


%% definition of the sys
s = tf('s');
G = 0.5 / (s^2+0.4*s+1);
sys = ss(G)

% constraints
u_min = -5; 
u_max = 5;

tsim = 15;       % time of the simulation
Ts = 0.1;   % seconds   tempo di sampling
xref = [0 0]';
x_0 = [0 10]'; % initial state

sysd = c2d(sys,Ts);
A = sysd.A;
B = sysd.B;
C = sysd.C;
D = sysd.D;
C_temp = eye(length(A));
D_temp = zeros(length(B), 1); 


%% 1) sim - LQR REGULATOR without oscillations
Q_LQ_1 = 10000*eye(2); 
R_LQ_1 = 1;
[LQ_d_1, Pr1, e1] = dlqr(A, B, Q_LQ_1, R_LQ_1 );  % would have been great if it weren't for saturations
% [K,Pr1,e] = dlqr(A,B,Q,R,N) 
% calculates the optimal gain matrix K 
% The default value N=0 is assumed when N is omitted.
%  returns the infinite horizon solution Pr1 of the associated discrete-time Riccati equation
% and the closed-loop eigenvalues e = eig(A-B*K)

Q_LQ_2 = 10*eye(2); 
R_LQ_2 = 1;
[LQ_d_2, Pr2, e2] = dlqr(A, B, Q_LQ_2, R_LQ_2 );

Q_LQ_3 = 0.1*eye(2); 
R_LQ_3 = 1;
[LQ_d_3, Pr3, e3]= dlqr(A, B, Q_LQ_3, R_LQ_3 );

% sim LQ_point_1


%% 2) MPC Q_sig  R_sig
% mpc take directly into account in the computation the constraints

% N = 10;
% Q_sig = blkdiag( kron(eye(N-1),Q_LQ_1) , S1);  % X è grosso N
% R_sig = kron(eye(N),R_LQ_1);


%% 3) MPC A_sig  B_sig

% A matrix
% Asig = A;
% for i = 2:N
%     Asig = [Asig; A^i];
% end
% 
% % B matrix
% Bsig = [];
% temp = [];
% for i = 1:N
%     temp = zeros(size(B,1)*(i-1),1);
%     for j = 0:N-i
%         temp = [temp; A^(j)*B];
%     end
%     Bsig = [Bsig temp];
% end
%             

%% 4) qudprog
% cost function of the MPC    Q_sig' X Q_sig + U R_sig U
% cost that accepts quadprog  0.5 U' H U + x_k' F U + 0.5 x_k' M x_k

% H = Bsig'*Qsig*Bsig + Rsig;
% M = Asig'*Qsig*Asig;
% F = Asig'*Qsig*Bsig;
% 0.5 not written as there is in all the term and
% minimising J or 2J does not change the result


%% 5) is it useful the term M?
% The term 1/2*x_k^T*M*x_k does not contain the control variable so it is
% not needed to solve optimization problem: remember that the goal of the
% optimization is to find the optimal control sequence
% x_k is the initial condition, is given I can't decide it


%% 
N1 = 5;
QMPC1 = Q_LQ_1; 
R1 = R_LQ_1;
S1 = Pr1; 

N2 = 100;
QMPC2 = Q_LQ_1; 
R2 = R_LQ_1;
S2 = Pr1;
% stessa Q e R e stesso terminal cost S
% solo il prediction horizon diverso per l'MPC

%%) sim regulator LQR discrete comparison
