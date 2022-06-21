clear
close all
clc
warning off 
%% SYSTEM DEFINITION
Gs=tf(0.5,[1 0.4 1])

%% POINT 1
Ts=0.1;
Gz=c2d(Gs,Ts);
pzmap(Gz)
[poles_G,zeros_G]=pzmap(Gz)
% Note that |poles| < 1 and draw the conclusion (asimptotically stable)

%% POINT 2
[A,B,C,D]=ssdata(Gz)

%% POINT 3
%Ybar=mu*Ubar   mu=gain of Gz, mu=Gz(1); (Final value theorem)
Ybar=0.5;
Ubar=inv(dcgain(Gz))*Ybar; %Ubar=1 

% x(k+1)=A*x(k)+B*u(k)
%Xbar=A*Xbar+B*Ubar --> (I-A)*Xbar=B*Ubar
Xbar=inv(eye(2)-A)*B*Ubar;


C_states=eye(2); % set the states all accessible (C_states=I)
D_states=[0 0]';

%% POINT 4
% dlqr sythetizes a discrete time LQR controller
% To check stability compute eigenvalues of the feedback system
Q = 100*eye(2);
R = 1;
Klqr1=dlqr(A,B,Q,R)

feedbackSystem = A-B*Klqr1;
poles_feedback=eig(feedbackSystem)
% Note that |poles_feedback| < 1 and draw the conclusion (asimptotically stable)

%alternatives
[Klqr1, P, CLP]=dlqr(A,B,Q,R) %it automatically computes the CL poles (CLP)

tsim=20;
% first initial state
x0=[5 20];


%set Xref=Xbar and introduce the equilibrium input in the control scheme!
xref=Xbar; 
sat_max=inf;
sat_min=-inf;
open('regulator_LQR_discrete_Ueq')
sim('regulator_LQR_discrete_Ueq')
keyboard
 
Q = eye(2);
R = 1;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=dlqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that |poles_feedback| < 1 and draw the conclusion (asimptotically stable)

tsim=10;
open('regulator_LQR_discrete_comparison_Ueq')
sim('regulator_LQR_discrete_comparison_Ueq')
keyboard

Q = eye(2);
R = 10;
disp(['Q: ' mat2str(Q)])
disp(['R: ' mat2str(R)])
[Klqr2,S,e]=dlqr(A,B,Q,R) % matlab already computes the eigenvalues (e)
% Note that |poles_feedback| < 1 and draw the conclusion (asimptotically stable)

tsim=10;
open('regulator_LQR_discrete_comparison_Ueq')
sim('regulator_LQR_discrete_comparison_Ueq')
keyboard

%% POINT 5

%extend the system to introduce the effect of noise acting on the system dynamics
%(v_x) 
% Use simulink to introduce the noise acting on the system output (v_y)

B_noise=[B ones(size(B,1),1)];
D_noise=[D_states zeros(size(D_states,1),1)];

%set power of v_x=0.001 in simulink block
power_vx=0.001
%set power of v_y=0.01 in simulink block
power_vy=0.01

% set Kalman filter parameters
QK=eye(2);          % variance of v_x
RK=10*QK(1);        % variance of v_y
NK=0;               % covariance v_x,v_y

x0K=1.5*x0;         %initial guess for x0

tsim=40;
open('regulator_LQG_discrete_Ueq')
sim('regulator_LQG_discrete_Ueq')
keyboard



%% POINT 6

%set power of v_x=0.01 in simulink block
power_vx=0.01
%set power of v_y=0.001 in simulink block
power_vy=0.001


%Tune the Kalman filter parameters 
RK=QK(1)*0.1; 
open('regulator_LQG_discrete_Ueq')
sim('regulator_LQG_discrete_Ueq')
keyboard

%% Final comparison

%set power of v_x=0.001 in simulink block
power_vx1=0.001;
%set power of v_y=0.01 in simulink block
power_vy1=0.01;

% set Kalman filter parameters
RK1=10*QK(1);        % variance of v_y

%set power of v_x=0.01 in simulink block
power_vx2=0.01;
%set power of v_y=0.001 in simulink block
power_vy2=0.001;


%Tune the Kalman filter parameters 
RK2=QK(1)*0.1; 
open('regulator_LQG_discrete_Ueq_comparison')
sim('regulator_LQG_discrete_Ueq_comparison')
