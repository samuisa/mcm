%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 
addpath('include/utils');

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();

%disp('iTj_0')
%disp(iTj_0)
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
q0 = [0 0 0 0 0 0 0]';

%% Define the tool frame rigidly attached to the end-effector
% Tool frame definition
psi = pi/10;
theta = 0;
phi = pi/6;

eRt = YPRToRot(psi, theta, phi);

disp('eRt');
disp(eRt);
e_r_te = [0.3; 0.1; 0];
eTt = [eRt, e_r_te; 0,0,0,1];

disp('eTt')
disp(eTt)

%% Initialize Geometric Model (GM) and Kinematic Model (KM)

% Initialize geometric model with q0
gm = geometricModel(iTj_0,jointType,eTt);

% Update direct geometry given q0
gm.updateDirectGeometry(q0)

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();

disp("eTt")
disp(eTt)
disp('bTt q = 0')
disp(bTt)

for j=1:gm.jointNumber
    bTi(:,:,j) = gm.getTransformWrtBase(j); 
end

figure
clf
grid on
hold on

title('CONFIGURATION for q=q0')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

az = 48;
el = 25;
view(az,el)

bri(:,1) = [0; 0; 0];

% Plot joints
for j = 1:gm.jointNumber
    bri(:,j+1) = bTi(1:3,4,j);              
end

bTt = gm.getToolTransformWrtBase();
bri(:,gm.jointNumber+2) = bTt(1:3,4); 

% plot joints
for j = 1:gm.jointNumber+2
    plot3(bri(1,j), bri(2,j), bri(3,j), 'bo')
end

% plot links
line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5);

hold off

%% Define the goal frame and initialize cartesian control
% Goal definition 
bOg = [0.2; -0.8; 0.3];
theta = pi/2;
bRg = YPRToRot(0,theta,0);
bTg = [bRg, bOg; 0, 0, 0, 1]; 
disp('bTg')
disp(bTg)

% control proportional gain
k_a = 0.8;
k_l = 0.8;

q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]';

gm.updateDirectGeometry(q)

bTt = gm.getToolTransformWrtBase();

disp('bTt q = q')
disp(bTt)

bT1 = gm.getTransformWrtBase(7);
disp('bT1')
disp(bT1)

for j=1:gm.jointNumber
    bTi(:,:,j) = gm.getTransformWrtBase(j); 
end

figure
clf
grid on
hold on

title('CONFIGURATION for q=q0')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

az = 48;
el = 25;
view(az,el)

bri(:,1) = [0; 0; 0];

% Plot joints
for j = 1:gm.jointNumber
    bri(:,j+1) = bTi(1:3,4,j);              
end

bTt = gm.getToolTransformWrtBase();
bri(:,gm.jointNumber+2) = bTt(1:3,4); 

% plot joints
for j = 1:gm.jointNumber+2
    plot3(bri(1,j), bri(2,j), bri(3,j), 'bo')
end

% plot links
line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5);

hold off


% Cartesian control initialization
cc = cartesianControl(gm, k_a, k_l);

%% Initialize control loop 

% Simulation variables
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

% preallocation variables
bTi = zeros(4, 4, gm.jointNumber);
bri = zeros(3, gm.jointNumber+1);

% joints upper and lower bounds 
qmin = -3.14 * ones(7,1);
qmin(6) = 0;
qmax = +3.14 * ones(7,1);
qmax(6) = 1;

show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

history.t = [];
history.x_dot_tool = [];
history.x_dot_ee = [];

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    gm.updateDirectGeometry(q);
    x_dot = cc.getCartesianReference(bTg);
    
    disp('x_dot');
    disp(x_dot);
    
    km.updateJacobian();
    
    %% INVERSE KINEMATICS
    q_dot = pinv(km.J)*x_dot;
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    % tool velocity computation
    r_ee_tool = gm.eTt(1:3,4);
    bTe = gm.getTransformWrtBase(gm.jointNumber);
    bRee = bTe(1:3,1:3);
    r_b = bRee * r_ee_tool;

    bJe = km.getJacobianOfLinkWrtBase(gm.jointNumber);
    
    x_dot_ee = bJe * q_dot;
    x_dot_tool = km.J * q_dot;

    disp('x_dot_ee');
    disp(x_dot_ee);

    disp('x_dot_tool');
    disp(x_dot_tool);

    list_x_dot_tool = x_dot_tool; 
    
    history.x_dot_tool = [history.x_dot_tool, list_x_dot_tool];
    history.t = [history.t, i];

    list_x_dot_ee = x_dot_ee; 
    
    history.x_dot_ee = [history.x_dot_ee, list_x_dot_ee];

    pm.plotIter(gm, km, i, q_dot);
    
    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    end
end

pm.plotFinalConfig(gm);
plotVelocities(history.t, history.x_dot_tool, history.x_dot_ee);