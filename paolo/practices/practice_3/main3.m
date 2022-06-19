% LQR controller & kalman filter

clear all; close all; clc    % 05_03    ora 1:09 

%% 1) discretise the system, poles and zeros
s = tf('s');
G = 0.5 / (s^2+0.4*s+1);
T_sampling = 0.1;   % seconds

G_d = c2d(G,T_sampling);
pzmap(G_d)
[poli, zeri] = pzmap(G_d)


%% 2) state space rappresentation
% final value theorem for discrete sys
% in prctice I apply a U_bar (step) and can know by knowing the gain of the
% sys (G(1)) the Y_bar   

G_1 = dcgain(G_d)   % gives me the gain og the sys
Y_bar = 0.5; 
U_bar = Y_bar / G_1

[A, B, C, D] = ssdata(G_d)
% x(k+1) = A x(k) + B u(k)
% at the eq x(k) = x(k+1)
X_bar = inv(eye(2)-A)*B*U_bar


%% 3) sim   LQR regulator around the eq point 0.5    (no more 0)
C_temp = eye(length(A))
D_temp = zeros(length(B), 1) % in questo caso ho un u solo
x_0 = [5 20]';
Q = 100*eye(2);
R = 1;
LQ_d = dlqr(A,B,Q,R)
% attenzione ai ref the LQ brings to 0 what enetrs so i make enter a
% difference of states, LQ returns the Delta_u to make the state remain in
% U_bar so the overall u will be Delta_u + U_bar that enters the real sys

% cl sys is AS?
closed_loop = A-B*LQ_d;
poles_cl = eig(closed_loop)


Q_2 = eye(2);
R_2 = 10;
LQ_d2 = dlqr(A,B,Q_2,R_2)


%% 5) not perfect model
% output much noisier than the states

% variables for the sim
power_vx=0.001
power_vy=0.01

% Kalman filter parameters (necessary for obtaining L)
Q_tilde = eye(2);        % var di Vx
R_tilde = 1;             % var i Vy
Z = 0;                   % covariance between Vx and Vy (simulink la chiama N)
x0_tilde = 1.5*x_0;        % Kalman filter initial guess (to start)

% il primpo temp era per far uscire dal blocco la x e non la y
% C_temp = eye(length(A))               
% D_temp = zeros(length(B), 1) 
% questo temp serve per gestire l'ingresso con anche il gaussian noise
B_temp2 = [B ones(length(B),1)];
D_temp2 = [D_temp zeros(length(D_temp),1)];
