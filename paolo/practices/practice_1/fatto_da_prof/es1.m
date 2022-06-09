close all
clear
clc
%% MULTIVARIABLE SYSTEM DESCRIPTION
%transfer function
den=conv([1 2],[1 3]);
G=tf({[1 1] 4 ; 0.5 2} , {den den ; den den});
%or also
s=tf('s');

G=[(s+1)/((s+2)*(s+3)) 4/((s+2)*(s+3));0.5/((s+2)*(s+3)) 2/((s+2)*(s+3))]

%% POINT 1
% Theorem 4.2.1. The characteristic polynomial Phi(s) associated with a 
% minimal realization of a system with transfer function matrix G(s) is the minimum
% common denominator of all the non null minors of any order of G(s).
% Poles are the roots of Phy(s)
% 
% Definition 4.4.3. The polynomial z(s) of the invariant zeros of G(s) is the
% polynomial with roots coinciding with all and only the invariant zeros of G(s).
% Theorem 4.4.2. The polynomial z(s) of the invariant zeros of G(s) is the
% greatest common divisor of all the numerators of all the minors of order r
% of G(s), where r is the normal rank of G(s), assuming that these minors are
% written so that they have the polynomial Phi(s) of the poles at the denominator. 
% r=2 -> so only one minor is analyzed for zeros computations

%compute poles
poles_model=pole(G)
%compute zeros
zeros_model=tzero(G)
%Note that zero should be exactly in 0, 4.8964e-16 is due to matlab
%numerical precision
% The system is asymptotically stable

%% POINT 2
poles_model11=pole(G(1,1))
poles_model12=pole(G(1,2))
poles_model21=pole(G(2,1))
poles_model22=pole(G(2,2))
%compute zeros
zeros_model11=tzero(G(1,1))
zeros_model12=tzero(G(1,2))
% complete it for all the Gij(s)



%% POINT 3
%Remark: norm2 of signal Asin(w*t+phi)is |A|, i.e. the amplitude of the
%sinusoid. The norm2 of a vector of sinusoidal signals concides with the
%norm2 of a vector with their amplitudes.

% Define the data of the signals and simulate the system G(s)
% U(t) = Usin(w*t+phi);  with U=[3 6], w=[2 2]; phi=[0 0]
U=[3 6];
w=[2 2];
phi=[0 0];

% [A, B, C, D] = ssdata(sys): given a LTI system sys (or transfer function), 
% returns the state spaces representation matrices A, B, C, D.
[A, B, C, D] = ssdata(G);
open('frequence_response_point3')
sim('frequence_response_point3')

% we address angular freq w_bar: both signals are purely sinusoidal
% with the same angular freq w_bar, i.e. their fourier transform
% has just the component at angular freq w_bar.
w_bar=2;
U_w_bar=U;
% compute the norm of the signals
normU_w_bar=norm(U_w_bar,2)

% Note that the sistem is LINEAR, therefore the output signals share 
% the angular frequency with the input ones!!!

% read amplitudes by the graph
Y_w_bar = [2.706    1.320] % read at t=20 and t=23.5
% or by the saved data
Y_w_bar = [max(Yt.Data(60:end,1)),max(Yt.Data(60:end,2))]
normY_w_bar=norm(Y_w_bar,2)

%compute the ratio of the norms2
gain=normY_w_bar/normU_w_bar
gaindB=20*log10(gain)

% sigma(sys): plot of the minimum and maximum singular values of the system sys
figure(3)
sigma(G)
hold on
grid on
plot(w_bar,gaindB,'ro','LineWidth',1.5)
hold off
keyboard
%% POINT 4
% Define the data of the signals and simulate the system G(s)
% U(t) = Usin(w*t+phi);  with U=[4 6]', w=[0 2]; phi=[-pi/2 0]
% Note that the constant is a cosine with w=0.

U=[4 6];
w=[0 2];

open('frequence_response_point4')
sim('frequence_response_point4')
keyboard % LOOK AT THE SIM 


% we address angular freq w_bar: the firts signal (constant)
% has no component at freq w_bar, therefore that 
% component has amplitude 0. 

w_bar=2;
U_w_bar=[0 U(2)]
normU_w_bar=norm(U_w_bar,2)

% read by the graph or use (ymax-ymin)/2 (amplitude of the sin is half of
% the total excursion) or ymax-y_w0

%compute amplitudes as (ymax-ymin)/2 (amplitude of the sin is half of the total excursion)
Y_w_bar = [(max(Yt.Data(60:end,1))-min(Yt.Data(60:end,1)))/2,...
    (max(Yt.Data(60:end,2))-min(Yt.Data(60:end,2)))/2]

%compute contribution at frequency w=0
Y_w_0=0.5*[max(Yt.Data(60:end,1))+min(Yt.Data(60:end,1)),max(Yt.Data(60:end,2))+min(Yt.Data(60:end,2))];
%compute amplitudes at w=w_bar removing the contribution at w=0;
Y_w_bar = [max(Yt.Data(60:end,1))-Y_w_0(1),max(Yt.Data(60:end,2))-Y_w_0(2)]

normY_w_bar=norm(Y_w_bar,2)
gain=normY_w_bar/normU_w_bar
gaindB=20*log10(gain)

% sigma(sys): plot of the minimum and maximum singular values of the system sys
figure(4)
sigma(G)
hold on
plot(w_bar,gaindB,'ro','LineWidth',1.5)
grid on
hold off

% there is the coupled term in the G(s) so the sinusoidal input influences 
% also the first output

keyboard
%% POINT 5
% Definition 4.4.3. The polynomial z(s) of the invariant zeros of G(s) is the
% polynomial with roots coinciding with all and only the invariant zeros of G(s).
% Theorem 4.4.2. The polynomial z(s) of the invariant zeros of G(s) is the
% greatest common divisor of all the numerators of all the minors of order r
% of G(s), where r is the normal rank of G(s), assuming that these minors are
% written so that they have the polynomial Phi(s) of the poles at the denominator.

% z=0 is an invariant zero for G(s)


% Define the data of the signals and simulate the system G(s)
% U(t) = Usin(w*t+phi);  with U=[1 -0.25]', w=[0 0]; phi=[-pi/2 -pi/2]

U=[1 -0.25];

open('frequence_response_point5')
keyboard % RUN SIM 

% we address angular freq w_bar=0: both signals are purely sinusoidal
% with the same angular freq w=w_bar=0, i.e. their fourier transform
% has just the component at angular freq w_bar=0;
w_bar=0;
U_w_bar=U;
normU_w_bar=norm(U_w_bar,2)

% read by the graph
Y_w_bar = [0  0]
% or by the saved data
Y_w_bar = [max(Yt.Data(60:end,1)),max(Yt.Data(60:end,2))]
% note that output signals are both = 0, but 1.0e-16 is due to matlab
% numerical precision
Y_w_bar = [0  0]
normY_w_bar=norm(Y_w_bar,2)
gain=normY_w_bar/normU_w_bar
gaindB=20*log10(gain)

% s=0 is an invariant zero for G(s) and gaindB=-Inf
% This is an example of the blocking property of invariant zeros
% Theorem 4.4.1. If lambda is an invariant zero, there exist an initial state x0 and a
% vector u0 such that, given an input u(t) = u0*exp(lambda*t); the output is y(t) = 0, t >= 0
%% POINT 6
% Theorem 2.2.1. The feedback system is I/O stable if
% ||S1||_inf ||S2||_inf < 1
% if the two systems S1 and S2 are linear, the following less restrictive condition 
% holds ||S1 S2||_inf < 1
R=eye(2)
normL=norm(G*R,inf)
% ||L||_inf < 1 ==> asimptotically stable
keyboard
%% POINT 7
%it is not possible to use the scheme with integrator because there is a
%zero in 0
R=[1/s 0;0 1/s];
L=G*R;

%Plot poles and zeros of the loop function L(s)
%Note that a forbidden cancellation of the invariant zero s=0 occours
figure()
pzmap(L) %pole and zero overlapped
poles_ol=pole(L)

zeros_ol=tzero(L)
%note that the poles are negative but I have a cancellation: 
%(zero = -9.7552e-17 = ~0) 
% Definition 5.2.1: a system composed of several blocks is as. stable if:
% (a) there are no hidden cancellations of unstable modes (NOT VERIFIED IN OUR CASE)
% (b) bounded inputs applied at any point of the system produce bounded 
%     outputs at any point of the system

%in the closed-loop function the cancellation is hidden (is seems as. stable)
Fin=((eye(2)+L))\L
F=minreal(Fin); 
keyboard

poles_cl=pole(F)
zeros_cl=tzero(F)

%% POINT 8
% cancelling the zero in 0 the control is not stable
t1=2;
A1=4;
t2=10;
A2=6;
open('frequence_response_integrators')
keyboard
% note output values (target not reached) and the control action

%% POINT 9
% modify the system as required
C=[1 1 0 0;0 0 1 1];
model2=ss(A,B,C,D)
poles_model2=pole(model2)
zeros_model2=tzero(model2)

%% POINT 10
%input signals definition
t1=2;
A1=4;
t2=10;
A2=6;
open('frequence_response_integrators')
% without derivative action in G(s) it is possible to use integrators to
% ensure perfect tracking of step inputs.



