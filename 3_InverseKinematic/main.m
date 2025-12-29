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
gm = geometricModel(iTj_0,jointType,eTt)

% Update direct geometry given q0
gm.updateDirectGeometry(q0)

% Initialize the kinematic model given the goemetric model
km = kinematicModel(gm)

bTt = gm.getToolTransformWrtBase()

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
bOg = [0.2; -0.7; 0.3];
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
t_end = 15.0;
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

%%%%%%% Kinematic Simulation %%%%%%%
for i = t
    % Updating transformation matrices for the new configuration 

    gm.updateDirectGeometry(q);

    % Get the cartesian error given an input goal frame
    x_dot = cc.getCartesianReference(bTg);
    disp('x_dot');
    disp(x_dot);
    disp('norma x_dot');
    disp(norm(x_dot(1:3)));
    disp(norm(x_dot(4:6)));

    % Update the jacobian matrix of the given model
    km.updateJacobian();
    % disp('km.J');
    % disp(km.J);

    %% INVERSE KINEMATICS
    % Compute desired joint velocities
    q_dot = pinv(km.J)*x_dot;

    % disp('q_dot')
    % disp(q_dot)

    % simulating the robot
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    % disp('q');
    % disp(q);

    x_dot_ac = km.J * q_dot;
    disp('x_dot_actual')
    disp(x_dot_ac)

    r_ee_tool = gm.eTt(1:3,4);

    bTe = gm.getTransformWrtBase(gm.jointNumber);
    bRee = bTe(1:3,1:3);

    r_b = bRee * r_ee_tool;

    v_ee = x_dot_ac(4:6);
    omega_ee = x_dot_ac(1:3);

    x_dot_ee = [omega_ee;
                v_ee];

    disp('ee velocity')
    disp(x_dot_ee)

    v_tool = v_ee + cross(omega_ee, r_b);
    omega_tool = omega_ee;

    x_dot_tool = [omega_tool; v_tool];
    disp('tool velocity')
    disp(x_dot_tool)

    pm.plotIter(gm, km, i, q_dot)

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01)
        disp('Reached Requested Pose')
        break
    else
        disp('Requested Pose Not Reached')

    end

end

pm.plotFinalConfig(gm);