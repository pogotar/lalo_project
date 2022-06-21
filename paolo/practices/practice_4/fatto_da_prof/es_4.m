clear
close all
clc

%% SYSTEM DEFINITION
Gs=tf(0.5,[1 0.4 1]);
% or
s = tf('s');
G = 0.5/(s^2 + 0.4*s +1);
sys = ss(G)

%constraints
umin = -5;
umax = 5;

%variables
tsim=15;
Ts = 0.1; %sampling time [s]
xref=[0 0]'; 
x0 = [0 10]'; %initial state

%discrete system and matrices
sysd = c2d(sys,Ts);
A = sysd.A
B = sysd.B;
C = sysd.C;
D = sysd.D;

C_states=eye(2); % set the states all accessible (C=[0, 1] --> C_states=I)
D_states=[0 0]';

%% POINT 1 - LQR REGULATOR without oscillations
Q_LQ = 10000*eye(2); 
R_LQ = 1;
[Kdlqr,Pr,E] = dlqr(A,B,Q_LQ,R_LQ);
S = Pr; %S is the terminal weight obtained through the LQR approach

sat_max=umax;
sat_min=umin;
KdlqrSat=Kdlqr;

Q_LQsat=Q_LQ/1000; %for oscillation reduction (at least no overshoot)
[KdlqrSat2,Pr_LQsat,E] = dlqr(A,B,Q_LQsat,R_LQ);
S2=Pr_LQsat;

% open('regulator_LQR_discrete_comparison')
% sim('regulator_LQR_discrete_comparison')
% keyboard

%% POINT 2 
N = 10; %prediction horizon
% Q matrix for Open-Loop MPC (with Q)
Qsig = blkdiag(kron(eye(N-1),Q_LQ),Pr);
Rsig = kron(eye(N),R_LQ);

%% POINT 3 - A and B matrices for the Open-Loop MPC
% A matrix
Asig = A;
for i = 2:N
    Asig = [Asig; A^i];
end

% B matrix
Bsig = [];
temp = [];
for i = 1:N
    temp = zeros(size(B,1)*(i-1),1);
    for j = 0:N-i
        temp = [temp; A^(j)*B];
    end
    Bsig = [Bsig temp];
end

%% B matrix bis

BB=[];
for i=1:N
    tmp=[zeros(size(B,1)*(i-1),size(B,2));B];
    for j=1:N-i
        tmp=[tmp;A^(j)*B];
    end
    BB=[BB tmp];
end


%% POINT 4
% H, F and S matrices (with Q)
H = Bsig'*Qsig*Bsig + Rsig;
M = Asig'*Qsig*Asig;
F = Asig'*Qsig*Bsig;

%% POINT 5
% The term 1/2*x_k^T*M*x_k does not contain the control variable so it is
% not needed to solve optimization problem: remember that the goal of the
% optimization is to find the optimal control sequence

%% POINT 6 - see mympc.m

%% POINT 7 - change prediction horizon N
N=5;
QMPC=Q_LQ; %optimal Q
R=R_LQ;
S=Pr; 

N2=100;
QMPC2=Q_LQ; %optimal Q
S2=S;
open('LinearMPC_noConstraints')
sim('LinearMPC_noConstraints')
keyboard
% using the final weight equal to infinite horizon Riccati matrix the
% prediction horizon does not affect the performance

%% POINT 8 - change prediction horizon N without LQ final weight
N=5;
S=Q_LQ;

N2=100;
S2=Q_LQ;
open('LinearMPC_noConstraints')
sim('LinearMPC_noConstraints')
keyboard

open('LinearMPC_noConstraints_comparison') %check N=100 is almost N=inf
sim('LinearMPC_noConstraints_comparison')
keyboard

N=5;
S=zeros(2);

N2=100;
S2=Pr;
open('LinearMPC_noConstraints')
sim('LinearMPC_noConstraints')
keyboard

open('LinearMPC_noConstraints_comparison') %check N=100 is almost N=inf
sim('LinearMPC_noConstraints_comparison')
keyboard


open('LinearMPC_noConstraints_comparison_N5') %check with N=5 difference between P=Q and P=0
sim('LinearMPC_noConstraints_comparison_N5')
keyboard

%% POINT 9
sat_max=umax;
sat_min=umin;
open('LinearMPC_constraints')
sim('LinearMPC_constraints')
% Constrained MPC performs slightly better than LQR (Uo is obtained known
% the saturation presence)

