%% State-space Model of Fuel Slosh
% 
% This code simulates lateral slosh in a high-G regime and its interaction 
% with vehicle attitude control. Typically, high-G slosh is modelled with 
% mechanical analogue models of a spring-mass model. 
% Equations come from "Saturn AS-501/S-IC flight control system design" by
% Frosch and Vallely
% The spring-mass-damper dynamic system is mathematically represented by as
% state-space model in which the state of the system is defined by a set of
% varaiables and their time derivatives. 

% Copyright 2022-2023 The MathWorks, Inc.
clear;close all;clc
%% Step 1: Open Model
% This can be done from the MATLAB UI, project shortcut, or MATLAB Command
% line. 

mdl = 'slosh_state_space';
open_system(mdl)

%% Step 2: Set Model Parameters
% Slosh mechanical model parameters are derived from fluids theory,
% experiments and CFD
% We are assuming a single slosh mass, n = 1
mSlosh = 1*10^3; % slosh mass (kg)
mVehicle = 1.6*10^4; % vehicle mass with engines and sloshing fluid (kg)
X_cg = 6; % center of gravity measured from gimbal (m)
X_s = 9; % slosh mass location measured from gimbal (m)
lSlosh = X_cg - X_s; % distance from C.G. to slosh mass (X_cg - X_s) (m)
I = 9*10^5; % moment of inertia with engines and sloshing fluid (kgm^2)
F = 9*10^5; % Total engine thrust (N)
Rprime = 9*10^5; % vectored engine thrust (N)
k3 = F/mVehicle; % (m/rad-s^2)
k4 = Rprime/mVehicle; % (m/rad-s^2)
omegaSlosh = 2; % slosh natural frequency (rad/s)
alphaSlosh = 0.1; % slosh damping slope (1/m)
xiSlosh = 0.3; % slosh equivalent linear damping ratio
c2 = Rprime*X_cg/I; % (1/s^2)
E = [eye(4) [zeros(3,2); 0 mSlosh/mVehicle]; [zeros(2,3) ...
    [0 1 -mSlosh*lSlosh/I; 1 -lSlosh 1]]] % mass matrix
A = [zeros(3) eye(3);[0 k3 0;0 0 mSlosh*k3/I] zeros(2);...
    [0 k3 -omegaSlosh^2 0 0 -2*xiSlosh*omegaSlosh]]; % matrix coefficient
B = [0 0 0 k4 -C2 0]; % matrix coefficient
C = % matrix coefficient
%% Step 4: Run simulation with slosh
% This can be done from Simulink or from the MATLAB command line.

sim(mdl)

