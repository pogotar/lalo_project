clear all
close all
clc

%% DATA

A = [-1.2822 0 0.98 0
    0 0 1 0
    -5.4293 0 -1.8366 0
    -128.2 128.2 0 0];

B = [-0.3
    0
    -17
    0];

C = [0 1 0 0
    0 0 0 1
    -128.2 128.2 0 0];
D = [0
    0
    0];

%definition of the system
sys = ss(A,B,C,D);

%initial conditions
x0 = [0 0 0 5000];

%limits of u
sat_max = 0.262; %[rad]
sat_min = -0.262; %[rad]
%limits of x(2)
x2_max = 0.349; %[rad]
x2_min = -0.349; %[rad]
%limits of slew rate
slew_max = 0.524; %[rad/s]
slew_min = -0.524; %[rad/s]

%proof of stability
pzmap(sys)
poles = pole(sys);
if real(poles) < 0
    fprintf('Open Loop System Stable')
else
    fprintf('Open Loop System Unstable')
end

%check for the observability and reachability of the system
% observability = obsv(sys);
% reachability = ctrb(sys);
% if rank(observability) == size(A,1) && rank(reachability) == size(A,1)
%     fprintf('System Observable and Reachable')
% else
%     keyboard
% end

C_states = eye(4);
D = [0 0 0 0]';
tsim = 30;

%% POINT 1
%we may have different attempts
%attempt 1

