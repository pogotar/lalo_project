clear
close all
clc
warning off 
%% SYSTEM DEFINITION
Gs=tf(0.5,[1 0.4 1])
%or
s=tf('s');
Gs=0.5/(s^2+0.4*s+1)

%% POINT 1
pzmap(Gs)
poles_G=pzmap(Gs)
% Note that Re(poles) < 0 and draw the conclusion (asimptotically stable)

%% POINT 2
[A,B,C,D]=ssdata(Gs)

%% POINT 3
% lqr sythetizes a continuous time LQR controller
% To check stability compute eigenvalues of the feedback system
Q = eye(2);
R = 1;
Klqr1=lqr(A,B,Q,R) %theorem 8.2.4

feedbackSystem = A-B*Klqr1;
poles_feedback=eig(feedbackSystem)
% Note that all poles_feedback < 0 and draw the conclusion (asimptotically stable)

%alternatives
[Klqr1, P, CLP]=lqr(A,B,Q,R) %it automatically computes the CL poles (CLP)

%% POINT 4
C_states=eye(2); % set the states all accessible (C=[0, 1] --> C_states=I)
D_states=[0 0]';

tsim=20;
% first initial state
x0=[0 10];
% % second initial state
%x0=[50 -40]';
% % third initial state
% x0=[-10 40]';

sat_max=inf;
sat_min=-inf;
open('regulator_LQR')
sim('regulator_LQR')
keyboard

%% POINT 5
Q = eye(2);
R = 100;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=lqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that all poles_feedback < 0 and draw the conclusion (asimptotically stable)

tsim=40;
open('regulator_LQR_comparison')
sim('regulator_LQR_comparison')
keyboard

Q = 100*eye(2);
R = 100;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=lqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that all poles_feedback < 0 and draw the conclusion (asimptotically stable)

tsim=40;
open('regulator_LQR_comparison')
sim('regulator_LQR_comparison') % identical output (the important value is the ratio between Q and R)
keyboard

Q = 10*eye(2);
R = 1;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=lqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that all poles_feedback < 0 and draw the conclusion (asimptotically stable)

tsim=20;
open('regulator_LQR_comparison')
sim('regulator_LQR_comparison') % best result
keyboard

%% POINT 6
Q = 10000*eye(2); %for oscillation reduction
R = 1;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=lqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that all poles_feedback < 0 and draw the conclusion (asimptotically stable)

tsim=20;
open('regulator_LQR_comparison')
sim('regulator_LQR_comparison') % best result
keyboard

%% POINT 7

%extend the system to introduce the effect of noise acting on the system dynamics
%(v_x) 
% Use simulink to introduce the noise acting on the system output (v_y)

B_noise=[B ones(size(B,1),1)];
D_noise=[D_states zeros(size(D_states,1),1)];

%set power of v_x=0.01 in simulink block
power_vx=0.01
%set power of v_y=0.01 in simulink block
power_vy=0.1

% set Kalman filter parameters
QK=eye(2);          % variance of v_x
RK=1;           % variance of v_y
NK=0;               % covariance v_x,v_y

x0K=10*x0;         %initial guess for x0

tsim=40;
open('regulator_LQG')
sim('regulator_LQG')
keyboard

open('regulator_LQG_comparisonKF')
sim('regulator_LQG_comparisonKF')
keyboard


%% POINT 8
%Tune the Kalman filter parameters to obtain a faster convergence
%Eigenvalues of (A-LC) must have Re{}<<<0, therefore the filter gain L must
%be high. This can be obtained with a small value of RK.

RK2=QK(1)*0.001; 
open('regulator_LQG_point8')
sim('regulator_LQG_point8')
keyboard

%% POINT 9
%Tune the Kalman filter parameters to obtain filtering of output noise.
% This can be obtained with a high value of RK (i.e. a large variance of v_y).

RK=QK(1)*100;               % variance of v_y
x0K=1.5*x0;         %initial guess for x0
tsim=tsim*10;

open('regulator_LQG')
sim('regulator_LQG')

