[Home](../index.md)

# Forward Kinematics

## Objective:
Upload the matrix  and DH placement of the frames, for the robots in the pdf

## How we solve the matrix multiplication
For the following problems, we used Matlab to solve all the matrices. The script calculates the matrices both individually (between each frame) and calculates the homogeneous transformation matrix between frame 0 and the frame associated with the final element of the robot. 

The Matlab script was the following: 

``` codigo
%% Denavit–Hartenberg (able Evaluator) 

% Computes individual link transforms (A_i) and cumulative transforms T_0_i. 

 

% Notes: 

% - Angles are in radians. 

% - This script can be configured for different types of joints. 

% -------------------- Local Function: Standard DH homogeneous transform -------------------- 

function H = dhStandard(a, alpha, d, theta) 

ct = cos(theta);  st = sin(theta); 

ca = cos(alpha);  sa = sin(alpha); 

H = [ ct, -st*ca,  st*sa, a*ct; 

      st,  ct*ca, -ct*sa, a*st; 

      0,      sa,     ca,    d; 

      0,       0,      0,    1 ]; 

end 

% -------------------- Local Function -------------------- 

clear; clc; 

% -------------------- Parameters -------------------- 

% Notes: 

% - In the case of joints with negligible distance between them, L = 0. 

l1 = 0; 

l2 = 1; 

l3 = 0; 

l4 = 1; 

l5 = 0; 

l6 = 1; 

% It is assumed that q = 0 for the initial pose. 

q = zeros(6,1); 


% -------------------- DH Table (Standard) -------------------- 

% Columns: a [m], alpha [rad], d [m], theta [rad] 

DH = table( ... 

    [0;   l2;  0;   0;   0;   0], ... 

    [pi/2;pi/2;-pi/2;pi/2;-pi/2;0], ... 

    [0;   0;   0;   l4;  0;   l6], ... 

    [pi/2+q(1); pi/2+q(2); q(3); q(4); q(5); -pi/2+q(6)], ... 

    'VariableNames', {'a','alpha','d','theta'} ); 


N = height(DH); 


% -------------------- Compute A_i and T_0_i -------------------- 

A = cell(N,1); % A{i} = A_i 

T = cell(N,1); % T{i} = T_0_i 

Tcum = eye(4); 

for i = 1:N 

    A{i} = dhStandard(DH.a(i), DH.alpha(i), DH.d(i), DH.theta(i)); 

    Tcum = Tcum * A{i}; 

    T{i} = Tcum; 

end

% -------------------- Display Results -------------------- 

disp("=== Evaluated DH Table (Standard DH) ==="); 

disp(DH); 


for i = 1:N 

    fprintf("\nA_%d =\n", i); 

    disp(A{i}); 

end 

T_0_N = T{end}; 

disp("=== T_0_N (end-effector pose) ==="); 

disp(T_0_N); 

``` 

## 1st exercise

Original problem image:



