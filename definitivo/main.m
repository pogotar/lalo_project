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

x2_max = deg2rad(20);
x2_min = deg2rad(-20);

over_shot = abs(0);
start_sign = sign(x0(4));

slewratemax = deg2rad(30);
slewratemin = deg2rad(-30);

%% 1)

T_sim = 15;

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

% sim_1

%% 2) 
sigma(sys)
[sv,w_out] = sigma(sys);

for i = 1:length(w_out)
    if sv(i) <= 1
        tep = i;
        break
    end
end

omega_taglio = w_out(i-1);
grid on
hold on
plot(omega_taglio,0,'*','color','r','LineWidth',1)
gain_taglio = sv(i-1);  
omega_sampling = 2 * omega_taglio;
T_sampling = 2*pi/omega_sampling;
Ts = T_sampling;

sysd = c2d(sys,T_sampling);

Q_d = Q4; 
R_d = R4;
[Klqr_d, P_d, CLP_d] = dlqr(sysd.A, sysd.B, Q_d, R_d);

% sim 2

%% 3)  multiple MPC cases no info on constraints

N1 = 15;
Q1 = Q_d; 
S1 = Q_d; 
R1 = R_d;
% P_d is the P_ of the infinite horizon with 
% Q_d = Q4 and R_d = R4

N2 = 15;
Q2 = Q_d; 
S2 = 100 * Q_d;
R2 = R_d;

N3 = 100;
Q3 = Q_d; 
S3 = Q_d; 
R3 = R_d;

N4 = 5;
Q4 = Q_d; 
S4 = P_d;
R4 = R_d;

%% 4) 

N5 = 100;
Q5 = Q_d; 
S5 = P_d;
R5 = R_d;

N6 = 100;
Q6 = 50 * Q_d; 
S6 = P_d;
R6 = R_d;

N7 = 100;
Q7 = Q_d; 
S7 = P_d;
R7 = 50 * R_d;


%% 5)
% sim


%% 6)

% sim

%% 7)
slewratemax = slewratemax*T_sampling;
slewratemin = slewratemin*T_sampling;

%sim

%% 8)

% x0 = [deg2rad(0) deg2rad(15) 0 -300];
x0K = [deg2rad(0) deg2rad(-20) 0 -280]; % initial guess for x0

% set power of v_x=0.01 in simulink block
power_vx = 1e-3;
% set power of v_y=0.001 in simulink block
power_vy = 1e-2;

B_noise = [sysd.B ones(size(sysd.B,1),1)];
D_noise = [D_states zeros(size(D_states,1),1)];

% set Kalman filter parameters
NK = zeros(4,3); % covariance v_x,v_y
QK = eye(4); % variance of v_x
RK = 0.001*eye(3); % variance of v_y

% open('MPC_KF') % ----------------------------------------------------------
% sim('MPC_KF') % -----------------------------------------------------------
% keyboard % ----------------------------------------------------------------

QK = blkdiag(1,1,1,1); % variance of v_x
RK = 10*blkdiag(1,1,1); % variance of v_y
