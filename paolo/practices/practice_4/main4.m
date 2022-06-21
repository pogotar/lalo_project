% MPC

clear all; close all; clc

% from 1) to 6) we assume to know the x

%% definition of the sys
s = tf('s');
G = 0.5 / (s^2+0.4*s+1)
tsim = 15;       % time of the simulation
x_0 = [0 10]'; % initial state
T_sampling = 0.1;   % seconds
G_d = c2d(G,T_sampling);
[A, B, C, D] = ssdata(G_d)
C_temp = eye(length(A));
D_temp = zeros(length(B), 1); 

u_min = -5; % constraints
u_max = 5;


%% 1) sim - LQR REGULATOR without oscillations
Q_LQ_1 = 10000*eye(2); 
R_LQ_1 = 1;

LQ_d_1 = dlqr(A, B, Q_LQ_1, R_LQ_1 );

Q_LQ_2 = 10*eye(2); %for oscillation reduction (at least no overshoot)
LQ_d_2 = dlqr(A, B, Q_LQ_2, R_LQ_1 );

% sim LQ_point_1
