clear all; close all; clc;

%% DATI

% x1 = angle of attack
% x2 = pitch angle
% x3 = pitch rate
% x4 = altitude
% y1 = pitch angle
% y2 = altitude
% y3 = speed

% Matrici del sistema
A = [-1.2822 0 0.98 0; 0 0 1 0; -5.4293 0 -1.8366 0; -128.2 128.2 0 0];
B = [-0.3 0 -17 0]';
C = [0 1 0 0; 0 0 0 1; -128.2 128.2 0 0];
D = [0 0 0]';
sys = ss(A,B,C,D);

% Condizioni iniziali sugli stati:
x0 = [deg2rad(0) deg2rad(-15) 0 200];

% Settare gli stati di riferimento per evitare i warning:
x_ref = [0 0 0 0]';

% Vincoli sull'ingresso U
sat_max = deg2rad(15); sat_min = deg2rad(-15);
% Vincoli sullo stato X2
x2_min = deg2rad(-20); x2_max = deg2rad(20);
% Vincoli sullo stato X4
over_shot_constraint = -0.01 * x0(4);
% Vincoli sullo slew rate dell'ingresso U
slewrateMax = deg2rad(30); slewrateMin = deg2rad(-30);

% Per vedere se il sistema è internamente stabile
poles_openloop = eig(A);
if real(poles_openloop) < 0
    fprintf('OPEN LOOP SYSTEM STABILE\n\n')
else
    fprintf('OPEN LOOP SYSTEM INSTABILE\n\n')
end

% Per vedere se il sistema è completamente osservabile e raggiungibile
if((rank(obsv(sys)) == size(A,1)) && rank(ctrb(sys)) == size(A,1))
    fprintf('SISTEMA OSSERVABILE E RAGGIUNGIBILE\n\n')
end

% Per rendere visibili tutti gli stati in uscita
C_states = eye(4);
D_states = [0; 0; 0; 0];

tsim = 10;

%% ES.1 LQR TEMPO CONTINUO

% Tentativo 1 (PITCH ANGLE > |20°|)
Q1 = 1*eye(4); R1 = 1;
[Klqr1, P1, CLP1] = lqr(A,B,Q1,R1); % CLOSED LOOP SYSTEM A.S.
% Tentativo 2 (PITCH ANGLE > |20°|)
Q2 = 100*eye(4); R2 = 1;
[Klqr2, P2, CLP2] = lqr(A,B,Q2,R2); % CLOSED LOOP SYSTEM A.S.
% Tentativo 3 (PITCH ANGLE > |20°|)
Q3 = eye(4); R3 = 100;
[Klqr3, P3, CLP3] = lqr(A,B,Q3,R3); % CLOSED LOOP SYSTEM A.S.
% Tentativo 4 (PITCH ANGLE < |20°|)
Q4 = 0.01*eye(4); R4 = 100;
[Klqr4, P4, CLP4] = lqr(A,B,Q4,R4); % CLOSED LOOP SYSTEM A.S.

% open('LQR_comparison') % ------------------------------------------------
% sim('LQR_comparison') % -------------------------------------------------
% keyboard % --------------------------------------------------------------

Klqr_LQ = Klqr4; Q_LQ = Q4; R_LQ = R4; P_LQ = P4;

%% ES.2 LQR TEMPO DISCRETO

% Per trovare il periodo di campionamento
sigma(sys)
grid on
hold on
SingularValues = findobj(gca, 'Type', 'line');
x = get(SingularValues, 'Xdata');
pulse = x{2,1};
y = get(SingularValues, 'Ydata');
module = y{2,1};
zero_dB = 0;
w_taglio = pchip(module,pulse,zero_dB); % 40.1521 [rad/s]
plot(w_taglio,zero_dB,'*','color','r','LineWidth',1)
legend('Singular Values','w_B')

% w_s > 2*w_taglio --> (2*pi)/T_s > 2*w_taglio --> T_s < pi/w_taglio con
% w_taglio = 2*pi/T_taglio --> T_s < T_taglio/2
T_taglio = 2*pi/w_taglio;
Ts = 0.5*(T_taglio/2);

slewrateMax = slewrateMax * Ts; slewrateMin = slewrateMin * Ts;

% Trasforma il sistema da continuo a discreto
sysd = c2d(sys,Ts);

Q_d = Q_LQ; R_d = R_LQ;
[Klqr_d, P_d, CLP_d] = dlqr(sysd.A,sysd.B,Q_d,R_d);
% open('LQR_discrete') % --------------------------------------------------
% sim('LQR_discrete') % ---------------------------------------------------
% keyboard % --------------------------------------------------------------

%% ES.3 MPC SENZA VINCOLI

Q_MPC = Q_d; R_MPC = R_d; P_MPC = Q_d;

SetN = [20 30 90];
N = SetN(3);

% open('MPC_noConstraints') % ---------------------------------------------
% sim('MPC_noConstraints') % ----------------------------------------------
% keyboard % --------------------------------------------------------------

P_MPC = P_d;
N = 1;

% open('MPC_noConstraints_opt') % -----------------------------------------
% sim('MPC_noConstraints_opt') % ------------------------------------------
% keyboard % --------------------------------------------------------------

N = SetN(3);

%% ES.4 MPC CON VINCOLI SU INGRESSO

%{
    Vincoli sull'ingresso U
    sat_max = deg2rad(15); sat_min = deg2rad(-15);
%}

% open('MPC_InputConstraints') % ------------------------------------------
% sim('MPC_InputConstraints') % -------------------------------------------
% keyboard % --------------------------------------------------------------

%{
    1) Is it still working?
        Sì
    2) Does it better than the LQR?
        E' leggermente migliore in quanto regola le sue azioni ottimali,
        tenendo conto dei limiti di saturazione, mentre l'LQ calcola le
        azioni ottimali e solo dopo le satura.
%}

Q_d = 100*Q_LQ; R_d = R_LQ;
[Klqr_d, P_d, CLP_d] = dlqr(sysd.A,sysd.B,Q_d,R_d);
Q_MPC = Q_d; R_MPC = R_d; P_MPC = P_d;

% open('MPC_InputConstraints') % ------------------------------------------
% sim('MPC_InputConstraints') % -------------------------------------------
% keyboard % --------------------------------------------------------------

%{
    3) How does it behave when you try to make it more aggressive?
        Aumentando la Q, quindi aumentando l'utilizzo del regolatore (le U
        aumentano), i due controllori peggiorano entrambi ed aumenta la
        differenza tra l'MPC e l'LQ
%}

% Li rimetto come prima
Q_d = Q_LQ; R_d = R_LQ;
[Klqr_d, P_d, CLP_d] = dlqr(sysd.A,sysd.B,Q_d,R_d);
Q_MPC = Q_d; R_MPC = R_d; P_MPC = P_d;

%% ES.5 MPC CON VINCOLI SU INGRESSO E X2

%{
    Se si utilizza la P_d (opt solution discreto) non funziona. Perchè?
    IDEA: Se fosse uguale alla soluzione ottimale, fin da subito (N = 1)
    allora i vincoli sul pitch angle non sarebbero rispettati (guarda il
    grafico sottostante)
%}

% open('MPC_X2Constraints') % ---------------------------------------------
% sim('MPC_X2Constraints') % ----------------------------------------------
% keyboard % --------------------------------------------------------------

%% ES.6 MPC CON VINCOLI SU INGRESSO, X2 E X4

%{
    overshot = - 1% * x4(0)
    x4(0) > 0  -->  x4(t) > overshot  --> -x4(t) < -overshot
    x4(0) < 0  -->  x4(t) < overshot
    -sign(x4(0))*x4(t) < -sign(x4(0))*overshot
    x4(0) = 200 --> -x4(t) < -overshot --> x4(t) > overshot
    x4(0) = -200 --> x4(t) < overshot
%}

% open('MPC_X4Constraints') % ---------------------------------------------
% sim('MPC_X4Constraints') % ----------------------------------------------
% keyboard % --------------------------------------------------------------

%% ES.7 MPC CON VINCOLI SU INGRESSO, X2, X4 E SLEW RATE DELL'INGRESSO

% open('MPC_AllConstraints') % --------------------------------------------
% sim('MPC_AllConstraints') % ---------------------------------------------
% keyboard % --------------------------------------------------------------

%% ES.8 KALMAN FILTER

tsim = 20;

x0K = [deg2rad(0) deg2rad(-20) 0 250]; % initial guess for x0

% set power of v_x=0.01 in simulink block
power_vx = 1e-3;
% set power of v_y=0.001 in simulink block
power_vy = 1e-2;

B_noise = [sysd.B ones(size(sysd.B,1),1)];
D_noise = [D_states zeros(size(D_states,1),1)];

% set Kalman filter parameters
NK = zeros(4,3); % covariance v_x,v_y
QK = blkdiag(1,1,1,1); % variance of v_x
RK = 0.001*blkdiag(1,1,1); % variance of v_y

% open('MPC_KF') % ----------------------------------------------------------
% sim('MPC_KF') % -----------------------------------------------------------
% keyboard % ----------------------------------------------------------------

QK = blkdiag(1,1,1,1); % variance of v_x
RK = 10*blkdiag(1,1,1); % variance of v_y

% open('MPC_KF') % ----------------------------------------------------------
% sim('MPC_KF') % -----------------------------------------------------------