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
sigma(sys)
grid on
xlim([10^(-10) 10^(10)])
ylim([-500 500])



% sysd = c2d(sys,Ts);
C_states = eye(length(A))
D_states = zeros(length(B), 1)