% This script initializes initial conditions for the trajectory of a drone
% in steady flight conditions (hovering) then calls the ODE45 function to
% numerically integrate the position of the drone with respect to the
% initial conditions, along with plotting the results answering Question
% 9
%
%   Author: Benjiman Smith
%   Collaborators: E. Owen, I. Quezada
%   Date: 1/25/2020
%
clc;
clear all;
m = 0.068; % mass of the drone [kg]
r = 0.06;  % body to motor distance [m]
k = 0.0024;  % [Nm/N]
rad = r/sqrt(2);  % [m]
g = 9.81; % gravity [m/s^2]
alpha = 2e-6;   % [N/(m/s)^2]
eta = 1e-3;    % [N/(rad/s)^2]     
Ix = 6.8e-5;   % moment of inertia in the x direction [kg m^2]
Iy = 9.2e-5;   % moment of inertia in the y direction [kg m^2]
Iz = 1.35e-4;  % moment of inertia in the z direction [kg m^2]
givens = [alpha eta Ix Iy Iz m r k rad g]; % givens vector

% Hover with disturbances
tspan = linspace(0,5); % time vector
Disturbance = randn(1, 3)/1000; % small pertubations
TrimForces = ones(1, 4) * m * g / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting

options = odeset('Events', @StopFnct, 'RelTol', 1e-8); % stop function that ends ODE when a tolerance of 1e-8 is met
[t, X] = ode45(@(t, F)SpecsCorrect(t, F, TrimForces, Disturbance, givens), tspan, conditions, options); % ODE Call
%Plotting
figure(4)
plot3(0, 0, -1, '.', 'MarkerSize',35);
set(gca, 'zdir', 'reverse');
hold on
plot3(X(:, 11), X(:, 10), X(:, 12),'linewidth', 2)
ylim([-6 6]);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Disturbed Drone (5 second Flight)')
legend('Origin', 'Trajectory');
hold off

load('RSdata_0958.mat');

xdata=rt_estimatedStates.signals.values(:,1);
ydata=rt_estimatedStates.signals.values(:,2);
zdata=rt_estimatedStates.signals.values(:,3);
figure(5)
plot3(xdata, ydata, zdata);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Experimental Data (Data 0958)');
figure(6)
psi = rt_estimatedStates.signals.values(:,4);
theta = rt_estimatedStates.signals.values(:,5);
phi =  rt_estimatedStates.signals.values(:,6);
rt_estimatedStates.signals.values(:,4);
times =rt_estimatedStates.time(:);
plot(times, psi,'linewidth', 2);
hold on
plot(times, theta, 'linewidth', 2);
plot(times, phi, 'linewidth', 2);
legend('\psi', '\theta', '\phi');
xlabel('time [s]')
ylabel('Radians')
title('Experimental bank, elevation, azimuth during time of flight');



