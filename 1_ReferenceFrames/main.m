addpath('include');

% TO DO: Test assignment 1 MCM 2024-2025

%%

%1.1 Angle-axis to rot

 h = [1,0,0]';
 theta = pi/2;
%
 h_1 = [0,0,1]';
 theta_1 = pi/3;
%
 p = [-pi/3;-pi/6;pi/3];
 h_2 = p/norm(p);
 theta_2 = norm(p);

 R1 = AngleAxisToRot(h, theta);
% 

%%

% 1.2 Rot to angle-axis

R2_2 = [1 0 0; 0 0 -1; 0 1 0]; 
R2_3 = [0.5 -sqrt(3)/2 0; sqrt(3)/2 0.5 0; 0 0 1];
R2_4 = [1 0 0; 0 1 0; 0 0 1]; 
R2_5 = [-1 0 0; 0 -1 0; 0 0 1];
R2_6 = [-1 0 0; 0 1 0; 0 0 1]; 
%
[h, theta] = RotToAngleAxis(R2_6) 

%%

% 1.3 Euler to rot

% psi=0; theta=0; phi=pi/2;
% psi=0; theta=0; phi=pi/3;
 psi=pi/3; theta=pi/2; phi=pi/4;
% psi=0; theta=pi/2; phi=-pi/12;

 R2 = YPRToRot(psi, theta, phi);

%%

% 1.4 Rot to Euler

 R4_2=[1 0 0; 0 0 -1; 0 1 0];
 R4_3=[1/2 -(sqrt(3)/2) 0; sqrt(3)/2 1/2 0; 0 0 1];
 R4_4=[0 -(sqrt(2)/2) sqrt(2)/2; 0.5 ((sqrt(2)*sqrt(3))/4) ((sqrt(2)*sqrt(3))/4); -(sqrt(3)/2) sqrt(2)/4 sqrt(2)/4];
% 
 [psi, theta, phi] = RotToYPR(R4_4)

%%

% 1.5 Rot to angle-axis with eigenvectors

    R5_1 = [1 0 0; 0 0 -1; 0 1 0];
    R5_2 = (1/9)*[4 -4 -7; 8 1 4; -1 -8 4];
%
    [h, theta] = RotToAngleAxisEig(R5_2)
    [h_2, theta_2] = RotToAngleAxis(R5_2)
