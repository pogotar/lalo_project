clear all; close all; clc;

%% DATA

% x1 = angle of attack
% x2 = pitch angle
% x3 = pitch rate
% x4 = altitude
% y1 = pitch angle
% y2 = altitude
% y3 = velocità * angolo di v rispetto all'orizzonte

% system matrices
A = [-1.2822  0      0.98     0
     0        0      1        0
     -5.4293  0      -1.8366  0
     -128.2   128.2  0        0];

B = [-0.3
      0
      -17
      0   ];

C = [0       1      0  0
     0       0      0  1
     -128.2  128.2  0  0];

D = [0
     0
     0];

sys = ss(A, B, C, D);
s = tf('s');

C_states = eye(length(A));
D_states = zeros(length(B), 1);

% initial states
x0 = [deg2rad(0) deg2rad(15) 0 -300];

% x ref
x_ref = [0 0 0 0]; % si considera come eq = 0 questio stati [0 0 128.2 5000]

% constraints
u_max = deg2rad(15);
u_min = deg2rad(-15);

%% 1) LQR continuous

T_sim = 10;

%first attempt
Q1 = 1*eye(length(A));
R1 = 1;
[k_LQ1, P, cl_poles] = lqr(A, B, Q1, R1); 

%second attempt
Q2 = 1000*eye(length(A));
R2 = 1;
[k_LQ2, P, cl_poles] = lqr(A, B, Q2, R2); 

%third attempt
Q3 = 1*eye(length(A));
R3 = 100;
[k_LQ3, P, cl_poles] = lqr(A, B, Q3, R3); 

%fourth attempt
Q4 = 1*eye(length(A));
R4 = 10000;
[k_LQ4, P4, cl_poles] = lqr(A, B, Q4, R4); 

% open("LQR_continuous.slx")
% sim("LQR_continuous.slx")
% keyboard

% choice
k_LQ = k_LQ4;
Q_LQ = Q4;
R_LQ = R4;
P_LQ = P4;

%% 2) LQR  discrete time

sigma(sys)
grid on
hold on
[sv,w_out] = sigma(sys);

for i = 1:length(w_out)
    if sv(i) <= 1
        tep = i;
        break
    end
end
omega_taglio = w_out(i);

plot(omega_taglio,0,'*','color','r')

% Shannon theorem
gain_taglio = sv(i);  %this is an approximation
omega_sampling = 2 * omega_taglio;
T_sampling = 2*pi/omega_sampling;

% discretization
sysd = c2d(sys,T_sampling);

Q_LQ_d = Q_LQ; 
R_LQ_d = R_LQ;
[k_LQ_d, P_LQ_d, CLP_d] = dlqr(sysd.A, sysd.B, Q_LQ_d, R_LQ_d);

% open('LQR_discrete.slx')
% sim('LQR_discrete.slx')
% keyboard

%% 3)
% P_MPC = P_d; S from the theory: the bound of MPC; to achive stationary
% solution for every N we set S = P stationary solution of the
% Riccati equation  

Ts = T_sampling; %to avoid undefined error
Q_MPC = Q_LQ_d;
R_MPC = R_LQ_d;
P_MPC = Q_LQ_d;

SetN = [20 30 90];

open('MPC_no_constraints_sim.slx')
sim('MPC_no_constraints_sim.slx')





