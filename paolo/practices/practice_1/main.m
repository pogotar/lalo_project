% MIMO
clear all; close all; clc

%% define a transfer function
s = tf('s');

G11 = (s+1) / ((s+1)*(s+3));
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

%% 3) simulink,  ||Y||_2 / ||U||_2
[A, B, C, D] = ssdata(G)
U = [3 6];   % ampiezza degli input
f = [2 2];   % frequenza degli input
phi = [0 0];   % frequenza degli input

norm_U = norm(U, 2)   % fa la norma 2 del vettore U


