% LQR controller & kalman filter

clear all; close all; clc

% from 1) to 6) we assume to know the x

%% 1) poles and zeros of L
s = tf('s');
G = 0.5 / (s^2+0.4*s+1);
poles_G = pole(G)
zeros_G = tzero(G)


%% 2) state space rappresentation of the system
[A, B, C, D] = ssdata(G);


%% 3) design LQR
% Q = I (da il peso alle x)   R = 1 (da il peso alle u)   cl eig are as?
Q = eye(2);
R = 1;
[k_LQ, P, cl_poles] = lqr(A, B, Q, R)      % P is the solution to the riccati equation
% k_LQ mi returna una 2x1 perchè il k moltiplica gli states non le u   (qui u ha dimensione 1)

% or otherwise
cl = A-B*k_LQ;  
poles_cl = eig(cl) % 2 method

% L = k_LQ*G;  % not correct dimensions
% F = (eye(2)+L)/L;
% F = minreal(F);   
% poles_cl = pole(F) % 3 method


%% 4) 5) 6) sym
% in the LQR I assume i want to put the reference to 0
x0 = [0 10];

% temporanee per poter avere in uscita dal blocco x (guarda il simulink)
% ho 2 uscite y
C_temp = eye(length(A))
D_temp = zeros(length(B), 1) % in questo caso ho un u solo


Q2 = 10*eye(2);                   
R2 = 1;                        
Q3 = eye(2);
R3 = 10;
% the one above has Q = eye(2)
%                   R = 1
k_LQ_2 = lqr(A, B, Q2, R2)
k_LQ_3 = lqr(A, B, Q3, R3)

sim('regulator_LQR_1')
open('regulator_LQR_1')


%% 7)  no measures of x
% but want to do the same things
