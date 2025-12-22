%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
disp('Q1.1');
iTj_0 = BuildTree();
disp('iTj_0');
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0, jointType);

%% Q1.3
disp('Q1.3');
bTe = geometricModel.getTransformWrtBase(geometricModel.jointNumber);
disp('bTe');
disp(bTe);
bT6 = geometricModel.getTransformWrtBase(6);
bT2 = geometricModel.getTransformWrtBase(2);

T_26 = inv(bT2) * bT6;
T_62 = inv(T_26);
disp('T_62');
disp(T_62);

%% Q1.4 Simulation
disp('Q1.4');

% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/4, 0, -pi/4, 0, 0.18, pi/5];
%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

%% Q1.5
disp('Q1.5');

km = kinematicModel(geometricModel);
bJ6 = km.getJacobianOfLinkWrtBase(6);
disp('bJ6');
disp(bJ6);
bJ7 = km.getJacobianOfLinkWrtBase(7);
disp('bJ7');
disp(bJ7);

%% Q1.6
disp('Q1.6');

km.updateJacobian();
disp('km.J')
disp(km.J);

%% Q1.7
clear geometricModel;

disp('Q1.7');

q = [0.7, -0.1, 1, -1, 0, 0.03, 1.3];
q_dot = [0.9; 0.1; -0.2; 0.3; -0.8; 0.5; 0];

iTj_0 = BuildTree();
jointType = [0 0 0 0 0 1 0];

geoModel = geometricModel(iTj_0, jointType);
geoModel.updateDirectGeometry(q);

kinModel = kinematicModel(geoModel);
bJn = kinModel.getJacobianOfLinkWrtBase(geoModel.jointNumber);
disp('bJn');
disp(bJn);

bTe = geoModel.getTransformWrtBase(geoModel.jointNumber);
bRn = bTe(1:3, 1:3);  % rotation matrix between last joint and the ee

bve = bJn * q_dot; % velocity of the ee wrt base in the base frame
disp('bve');
disp(bve);

r = bTe(1:3, 4); % distance between last joint and the base
r_cross = vecToSkew(r);   

Ad_inv = [bRn, zeros(3); r_cross * bRn, bRn];

eve = inv(Ad_inv) * bve;   % velocità EE frame
disp('eve');
disp(eve);