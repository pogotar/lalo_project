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
% open('regulator_LQR_1')


%% 7)  LQG 
% no measures of x
% but want to do the same things
% il mio modello è impreciso, non tiene conto dei white noise (non ci sono nel blocco)

% variables for the sim
power_vx=0.01
power_vy=0.1

% Kalman filter parameters (necessary for obtaining L)
Q_tilde = eye(2);        % var di Vx
R_tilde = 1;             % var i Vy
Z = 0;                   % covariance between Vx and Vy (simulink la chiama N)
x0_tilde = 10*x0;        % Kalman filter initial guess (to start)

% il primpo temp era per far uscire dal blocco la x e non la y
% C_temp = eye(length(A))               
% D_temp = zeros(length(B), 1) 
% questo temp serve per gestire l'ingresso con anche il gaussian noise
B_temp2 = [B ones(length(B),1)];
D_temp2 = [D_temp zeros(length(D_temp),1)];

% in Kalman ci vanno i normali A,B,C,D


%% 8) faster convergence of the kalman filter 
% modify R_tilde 
% pro: se + piccola L più grossa, autovalore dell' errore + negativo, dinamica 
%      dell'arrore tra x e xpredetta più corta, kalman filter da una buona
%      predizione della x più velocemente
% con: L + grossa, il kalman filter darà più peso a y nella predizione di x
%      questo vuol dire che se y fa schifo anche la predizione, sarà più
%      veloce ma farà schifo

R_tilde2 = R_tilde*0.001; 

% Q gives me an idea of how much my rappresentation of the sys is good

% for example
% if I know that my y is affected by a noise that is 10 times the noise on
% the x I could set the R = 10*Q per informare che mi fido molto poco del
% segnale y


%% 9) reduce meas noise on state espimation
R_tilde3 = R_tilde*100; 


% open sim regulator LQG_2

% 05_03    ora 1:09   arriva alla fine della practice 2