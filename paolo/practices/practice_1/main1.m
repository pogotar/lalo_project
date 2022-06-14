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
% I have a zero in zero, I have to be careful not to put integrators
% otherwise (see point 8) I will obtain an unstable sys


%% 1,2) poles and zeros  
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


%% 3) simu_1  U = [sin sin]  gain = ||Y||_2 / ||U||_2
[A, B, C, D] = ssdata(G)
U1_max = [3 6];   % ampiezza degli input
f1 = [2 2];   % frequenza degli input
phi1 = [0 0];   % frequenza degli input

norm_U1 = norm(U1_max, 2)   % fa la norma 2 del vettore U

% open('frequence_response')
out_sim1 = sim('frequence_response_1')   % sim esegue il simulink,   out_sim = prende l'output del sim e lo da alla var out_sim

% to have te norm of Y i have to not take into account transitory so I
% start after 60 time instants

Y1_max = [max( out_sim1.Yt.Data(60:end,1)),  max(out_sim1.Yt.Data(60:end,2))]  
% max(Yt.Data(60:end,1)) prende il massimo dei valori di Yt           
% .Data per accedere ai valori nel tempo        60 per scartare i primi 60
% :end per dire di andare fino alla fine        
% ,1 per prendere la prima colonna di output (u_1)  

norm_Y1 = norm(Y1_max, 2)   % fa la norma 2 del vettore Y

gain1 = norm_Y1 / norm_U1
gain_dB1 = 20*log10(gain1);

% sigma plots the minimum and maximum singular values of the sys
figure(1)
sigma(G)
hold on; grid on
plot (f1(2), gain_dB1, 'ro','LineWidth',1.5)  
% ho checkato che se immetto 2 frequenze a f = 2 il guadagno è minore del
% maggiore dei singular value "autovalori"
% e meggiore del minore dei singular values


%% 4) simu_2  U = [cost sin]  gain = ||Y||_2 / ||U||_2
U2_max = [4 6];   % ampiezza degli input
f2 = [0 2];   % frequenza degli input
phi2 = [0 0];   % frequenza degli input

norm_U2 = norm(U2_max, 2)  

out_sim2 = sim('frequence_response_2')  

% siccome c'è uno step la sinsoide in otput non è centrata, per trovare
% ampiezza (max-min) / 2
Y2_max = [(max( out_sim2.Yt.Data(60:end,1) ) - min( out_sim2.Yt.Data(60:end,1) )) / 2 ,  ...
          (max( out_sim2.Yt.Data(60:end,2) ) - min( out_sim2.Yt.Data(60:end,2) )) / 2   ]  
 
norm_Y2 = norm(Y2_max, 2)  

gain2 = norm_Y2 / norm_U2
gain_dB2 = 20*log10(gain2);

figure(2)
sigma(G)
hold on; grid on
plot (f2(2), gain_dB2, 'ro','LineWidth',1.5)  
% both output sinusoidal since G is not diagonal


%% 5) simu_3  U = [cost cost]  gain = ||Y||_2 / ||U||_2
U3_max = [1 -0.25];   % ampiezza degli input
f3 = [0 0];   % frequenza degli input
phi3 = [0 0];   % frequenza degli input

norm_U3 = norm(U3_max, 2)  

out_sim3 = sim('frequence_response_3')  

% siccome c'è uno step la sinsoide in otput non è centrata, per trovare
% ampiezza (max-min) / 2
Y3_max = [ max( out_sim3.Yt.Data(60:end,1)) , max( out_sim3.Yt.Data(60:end,2))]  % all'infinito l'output è zero ->
% the cause is the blocking property of the zeros but only in some
% directions example try the U_3 under here
 
U3_max = [4 6];   % ampiezza degli input
out_sim3 = sim('frequence_response_3')  
Y3_max = [ max( out_sim3.Yt.Data(60:end,1)) , max( out_sim3.Yt.Data(60:end,2))]  % now the Y is not 0


%% 6)  R(s) = I  is cl AS?
% use small gain theorem  (infinite norm)
% ||L||_inf < 1   then F is AS
R = eye(2);
norm_L6 = norm(G*R,inf) % va a 0.7683


%% 7) R = 1/s * eye(2)   cl stab?
R = 1/s*eye(2);
L = R*G;
norm_L7 = norm(L,inf) % va a inf
% not stable if I have a zero in zero and I put a pole in zero there are
% cancellations, there is this theorem
% a sys composed by several blocks is AS if:
% - no hidden cancellations of unstable modes (here we do not satisfy)
% - bounded inputs applied at any points of the sys produce bounded outputs
%   at any point of the sys 

% let's see the cl

F = (eye(2)+L)/L;
F = minreal(F);    % cancels almost cancelations
poles_F = pole(F) % every pole is zero (there is one positive but it is an error, it is super small)
% but I can't expect stability (or goog tracking of the reference signal)
% cuse of the comment before (check also point 8)
zeros_F = tzero(F)


%% 8) simu_4  R = 1/s * eye(2)   cl stab?

% check frequence response 4
% imagine a tank with no bottom and I try to stabilise the level with a
% pump

%% 9) 10) pick the old sys and change C
% teacher purposefully makes me do it so that I won't have a zero in zero
% and so a controller with an integrator in closed loop will give me no
% prob

C_new = [1 1 0 0
         0 0 1 1];

G_new = ss(A, B, C_new, D);
poles_G_new = pole(G_new) % no zero in zero
zeros_G_new = tzero(G_new)

% now try the integrators and should be fine 
% check frequence response 5
