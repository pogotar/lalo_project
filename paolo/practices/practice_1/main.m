% MIMO introduction
clear all; close all; clc

%% define a transfer function
s = tf('s');

G11 = (s+1) / ((s+2)*(s+3));
G12 = 4 /((s+2)*(s+3));
G21 = 0.5 /((s+2)*(s+3));
G22 = 2 /((s+2)*(s+3));

G = [G11 G12
     G21 G22];

%% 1,2) poles and zeros point 
poles_G = pole(G)   % -3 -3 -3 -2 -2 -1
zeros_G = tzero(G)  % -3 -1 -1 

poles_G11 = pole(G11) % -3 -1 
zeros_G11 = tzero(G11) % -1

poles_G12 = pole(G12) % -3 -2
zeros_G12 = tzero(G12) % 

poles_G21 = pole(G21) % -3 -2
zeros_G21 = tzero(G21) % 

poles_G22 = pole(G22) % -3 -2
zeros_G22 = tzero(G22) %

%% 3) simu  gain of the sys ||Y||_2 / ||U||_2
[A, B, C, D] = ssdata(G)
U_max = [3 6];   % ampiezza degli input
f = [2 2];   % frequenza degli input
phi = [0 0];   % frequenza degli input

norm_U = norm(U_max, 2)   % fa la norma 2 del vettore U

% open('frequence_response')
out_sim = sim('frequence_response')   % sim esegue il simulink,   out_sim = prende l'output del sim e lo da alla var out_sim

% to have te norm of Y i have to not take into account transitory so I
% start after 60 time instants

Y_max = [max( out_sim.Yt.Data(60:end,1)),  max(out_sim.Yt.Data(60:end,2))]  
% max(Yt.Data(60:end,1)) prende il massimo dei valori di Yt           
% .Data per accedere ai valori nel tempo        60 per scartare i primi 60
% :end per dire di andare fino alla fine        
% ,1 per prendere la prima colonna di output (u_1)  

norm_Y = norm(Y_max, 2)   % fa la norma 2 del vettore Y

gain = norm_Y / norm_U
gain_dB = 20*log10(gain);

% sigma plots the minimum and maximum singular values of the sys
figure(1)
sigma(G)
hold on; grid on
plot (f(1), gain_dB, 'ro','LineWidth',1.5)  
% ho checkato che se immetto 2 frequenze a f = 2 il guadagno è minore del
% maggiore dei singular value "autovalori"
% e meggiore del minore dei singular values