% This script initializes initial conditions for the trajectory of a drone
% in steady flight conditions (5m/s east) then calls the ODE45 function to
% numerically integrate the position of the drone with respect to the
% initial conditions, along with plotting the results answering Question
% 8
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
R = r/sqrt(2);  % [m]
g = 9.81; % gravity [m/s^2]
alpha = 2e-6; % [N/(m/s)^2]
eta = 1e-3; % [N/(rad/s)^2]
Ix = 6.8e-5; % moment of inertia in the x direction [kg m^2]
Iy = 9.2e-5; % moment of inertia in the y direction [kg m^2]
Iz = 1.35e-4; % moment of inertia in the z direction [kg m^2]
givens = [alpha eta Ix Iy Iz m r k R g]; % givens vector

%% East Trim
V = 5; % Velocity in east
tspan = linspace(0,10); % time vector
Pertubations = zeros(1, 3); % No perturbation
solveF = @(x) m*g*sin(x) - 25*eta*cos(x).^2; % symbolics to solve for phi and f_mag
phi = fzero(solveF, 0);
f_Mag = m*g*cos(phi) + 25*eta*sin(phi).^2;
TrimForces = ones(1, 4) * f_Mag / 4; % forces required by each motor to maintain altitude with 5m/s east vel
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting
conditions(2) = V*cos(phi); % y velocity component
conditions(3) = -V*sin(phi); % z velocity component
conditions(7) = phi; % set phi

options = odeset('Events', @StopFnct, 'RelTol', 1e-10); % stop function that ends ODE when a tolerance of 1e-8 is met
[t, X] = ode45(@(t, F)SpecsCorrect(t, F, TrimForces, Pertubations, givens), tspan, conditions, options); % ODE call
%plotting
figure(2)
plot3(0, 0, -1, '.', 'MarkerSize',35);
grid on
set(gca, 'zdir', 'reverse');
hold on
plot3(X(:, 11), X(:, 10), X(:, 12),'linewidth', 2)
zlim([-6 6]);
ylim([-6 6]);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Steady 5 m/s East trim (10 second flight)')
legend('Origin', 'Trajectory');
hold off


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% East Trim With xsi = 90 (elevation)
phi =0;
V = 5;
tspan = linspace(0,10); % time vector
Pertubations = zeros(1, 3); % No perturbation
solveF = @(x) m*g*sin(x) - 25*eta*cos(x).^2;
Theta = fzero(solveF, 0);
f_Mag = m*g*cos(Theta) + 25*eta*sin(Theta).^2;
Theta = -Theta;
TrimForces = ones(1, 4) * f_Mag / 4; % forces required by each motor to maintain hover
conditions = zeros(1, 12); % initialize conditions vector (very large)
conditions(12) = -1; % set down direction to 1 to make signs correct for plotting
conditions(1) = V*cos(Theta);
conditions(3) = V*sin(Theta);
conditions(8) = Theta; % initialize theta
conditions(9) = (pi/2); % set azimuth to 90 deg

options = odeset('Events', @StopFnct, 'RelTol', 1e-10); % stop function that ends ODE when a tolerance of 1e-8 is met
[t, X] = ode45(@(t, F)SpecsCorrect(t, F, TrimForces, Pertubations, givens), tspan, conditions, options); % ODE call

figure(3)
plot3(0, 0, -1, '.', 'MarkerSize',35);
grid on
set(gca, 'zdir', 'reverse');
hold on
plot3(X(:, 11), X(:, 10), X(:, 12), 'linewidth', 2)
zlim([-6 6]);
ylim([-6 6]);
zlabel('Down Position, [m]')
xlabel('East Position, [m]')
ylabel('North Position, [m]')
title('Steady 5 m/s East trim \psi = 90^{\circ} (10 second flight)')
legend('Origin', 'Trajectory');

