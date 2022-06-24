clear all; close all; clc;

%% DATI

% x1 = angle of attack
% x2 = pitch angle
% x3 = pitch rate
% x4 = altitude
% y1 = pitch angle
% y2 = altitude
% y3 = velocità * angolo di v rispetto all'orizzonte

% matrici del sistema
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

sys = ss(A, B, C, D)
s = tf('s');
% sigma(sys)
% grid on
% xlim([10^(-10) 10^(10)])
% ylim([-500 500])

C_states = eye(length(A));
D_states = zeros(length(B), 1);

% stati iniziali
x0 = [deg2rad(0) deg2rad(15) 0 -300];

% riferimento
x_ref = [0 0 0 0]; % si considera come eq = 0 questio stati [0 0 128.2 5000]

% vincoli
u_max = deg2rad(15);
u_min = deg2rad(-15);

%% 1)

T_sim = 40;

Q1 = 1*eye(length(A));
R1 = 1;
[k_LQ1, P, cl_poles] = lqr(A, B, Q1, R1); 

Q2 = 1000*eye(length(A));
R2 = 1;
[k_LQ2, P, cl_poles] = lqr(A, B, Q2, R2); 

Q3 = 1*eye(length(A));
R3 = 100;
[k_LQ3, P, cl_poles] = lqr(A, B, Q3, R3); 

Q4 = 1*eye(length(A));
R4 = 10000;
[k_LQ4, P, cl_poles] = lqr(A, B, Q4, R4); 


%% 2) 
sigma(sys)
[sv,w_out] = sigma(sys);

for i = 1:length(w_out)
    if sv(i) <= 1
        tep = i;
        break
    end
end

omega_taglio = w_out(i)
gain_taglio = sv(i);  % non proprio 1 ma vabbe
omega_sampling = 2 * omega_taglio;
T_sampling = 2*pi/omega_sampling;

sysd = c2d(sys,T_sampling);

Q_d = Q4; 
R_d = R4;
[Klqr_d, P_d, CLP_d] = dlqr(sysd.A, sysd.B, Q_d, R_d);

% sim 2


