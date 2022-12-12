% RVCTransMatrix
startup_rvc;

% Calculate the revolute link transformation
thetaDeg = [10 50 65];
d = [32 0 0];
a = [0 11 16];
alphaDeg = [90 0 0];
theta = deg2rad(thetaDeg);
alpha = deg2rad(alphaDeg);


len = 3;
p1 = [0; 0; 0];

for i = 1:len
    % offset: the robot does not start with home position, set to 0 theta angle
    % if it starts from home position
    L(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
end

robot = SerialLink(L, 'name', 'revolute');
% T is 0-T-n
T = robot.fkine([theta]) 
p0 = T*p1

%% Similarity Transformation 
startup_rvc;

R_01 = roty(pi/2)
A = rotz(pi/4)
R_10 = (R_01)^(-1)
B = R_10 * A * R_01

%% Theta, K; Axis Angle Representation
startup_rvc;
R = rotx(60, 'deg') * roty(30, 'deg') * rotz(90, 'deg');
theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2)
k = 1/(2*sin(theta))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]

%% Calculate Joint Velocity (Jacobian, Inverse Kinematics)
% RVCTransMatrix
startup_rvc;

% Calculate the revolute link transformation
thetaDeg = [45 170];
d = [0 0];
a = [0.5 0.5];
alphaDeg = [0 0];
theta = deg2rad(thetaDeg);
alpha = deg2rad(alphaDeg);

len = 2;
p1 = [0; 0];

for i = 1:len
    % offset: the robot does not start with home position, set to 0 theta angle
    % if it starts from home position
    L(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
end

robot = SerialLink(L, 'name', 'two link');
T = robot.fkine(theta)

% Calculate Jacobian at configuration qi
J = robot.jacob0(theta)
% Take subset of Jacobian for just x-y position
Jxy = J(1:2, :)
% Angular Velocity ([0.1 0] represents for [x_dot y_dot])
q_dot = inv(Jxy)*[0.1 0]'

%% Quinc Polynomial Trjectories
close all;clear variables; clc;
startup_rvc;
dbstop if error

time = 0:0.1:5; % 0~5s
figure

% [trajectory, velocity, acceleration] = 
% tpoly(start posittion, end position, time array, start velocity, end velocity
% here start position: 0 rad, end position 1 rad
[S, SD, SDD] = tpoly(0, 1, time, 0, 0);

subplot(3, 1, 1)
plot(time, S)
subplot(3, 1, 2)
plot(time, SD)
subplot(3, 1, 3)
plot(time, SDD)

%% Quintic Polynomial Trajectory for 6 DOF robot
close all;clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560

% Step1: Find the translation matrix
T1 = transl(0.4,0.2,0)*trotx(pi);
T2 = transl(0.4,-0.2,0)*trotx(pi/2);

% Step2: Inverse Kinematics for 6-axis spherical wrist revolute robot
pose1 = p560.ikine6s(T1);
pose2 = p560.ikine6s(T2);

% Step3: Generate trajectory matrix
t = 0:0.01:5;
s = mtraj(@tpoly, pose1, pose2, t);
sd = [zeros(1, 6); diff(s)];
sdd = [zeros(1, 6); diff(sd)];
sddd = [zeros(1, 6); diff(sdd)];

f1 = figure
subplot(4, 1, 1)
plot(t, s); grid on; xlim([min(t) max(t)]);
title('trajectory')
legend('q1', 'q2','q3','q4','q5','q6')
subplot(4, 1, 2)
plot(t, sd); grid on; xlim([min(t) max(t)]);
title('velocity')
subplot(4, 1, 3)
plot(t, sdd); grid on; xlim([min(t) max(t)]);
title('acceleration')
subplot(4, 1, 4)
plot(t, sddd); grid on; xlim([min(t) max(t)]);
title('jerk (rad/s/s/s)')

% End effector position
% Find the translation matrix by using joint positions
T = p560.fkine(s);
% Find only the [x,y,z] positions, translational components
p = transl(T);
% Plot (x,y) of the 
plot(p(:,1),p(:,2));axis equal; xlabel('X(m)'); ylabel('Y(m)'); grid on;
title('Locus of joint space path')

%% Trapezoidal Trajectory
close all;clear variables; clc;
startup_rvc;
dbstop if error

time = 0:0.1:18;
% [S, SD, SDD] = lspb(0, 1, time); % by default tb = time/3
[S, SD, SDD] = lspb(0, 1, time, 0.11); % V = 0.06 rad/s
figure
subplot(3, 1, 1)
plot(S)
subplot(3, 1, 2)
plot(SD)
subplot(3, 1, 3)
plot(SDD)
%% Trapezoidal Trajectories for 6DOF robot PUMA
close all; clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560
% Translate by (0.4, 0.2, 0) and rotate by π about x-axis
T1 = transl(0.4, 0.2, 0) * trotx(pi);
%  Translate by (0.4, -0.2, 0) and rotate by π/2 about x-axis
T2 = transl(0.4, -0.2, 0) * trotx(pi/2);
% Travel from T1 to T2 
pose1 = p560.ikine6s(T1);
pose2 = p560.ikine6s(T2);
% time b = 5/3; Thus, t = 5
t = [0:0.01:5];

f2 = figure(1);
% Generate path with multiple poses
s = mtraj(@lspb, pose1, pose2, t);
sd = [zeros(1,6); diff(s)];
sdd = [zeros(1,6); diff(sd)];
sddd = [zeros(1,6); diff(sdd)];

subplot(4,1,1)
plot(t, s); grid on; xlim([min(t), max(t)]);
title('LSPB trajectory');
xlabel('Time');
ylabel('Position [rad]')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
subplot(4,1,2)
plot(t, sd); grid on; xlim([min(t), max(t)]);
xlabel('Time'); 
ylabel('Velocity [rad/s]')
subplot(4,1,3)
plot(t, sdd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Acceleration [rad/s/s]')

%% Quintic polynomial Trajectories
close all; clear variables; clc;
startup_rvc;
dbstop if error
time = 0:0.1:5;%%% 5 sec

figure
% Start from 0 rad to 1 rad
% velocity limit from 5 to 0
[S,SD,SDD] = tpoly(0, 1, time,5,0);
%%%%% [S,SD,SDD] = tpoly(0, 1, time); by default the boundaries conditions for velocities are V_i = V_f=0
subplot(3,1,1)

plot(time,S)  %% position trajectory
subplot(3,1,2)
plot(time,SD) %% Velocity
subplot(3,1,3)
plot(time,SDD) %Acceleration

%% Quintic polynomial Trajectories with translation
close all; clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560
T1 = transl(0.4, 0.2, 0) * trotx(pi)
T2 = transl(0.4, -0.2, 0) * trotx(pi/2)

pose1 = p560.ikine6s(T1);  %%inverse kinematics for 6-axis spherical wrist revolute robot
pose2 = p560.ikine6s(T2);


figure(1)
subplot(2,1,1)
p560.plot(pose1);

t = [0:0.01:5];
s = mtraj(@tpoly, pose1, pose2, t);
sd = [zeros(1,6); diff(s)];
sdd = [zeros(1,6); diff(sd)];
%sddd = [zeros(1,6); diff(sdd)];

%Show position, velocity and acceleration as a function of time
f1 = figure(2);
subplot(4,1,1)
plot(t, s); grid on; xlim([min(t), max(t)]);
title('Quintic polynomial trajectory');
xlabel('Time');
ylabel('Position [rad]')
legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
subplot(4,1,2)
plot(t, sd); grid on; xlim([min(t), max(t)]);
xlabel('Time'); 
ylabel('Velocity [rad/s]')
subplot(4,1,3)
plot(t, sdd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Acceleration [rad/s/s]')
% subplot(4,1,4)
% plot(t, sddd); grid on; xlim([min(t), max(t)]);
% xlabel('Time');
% ylabel('Jerk [rad/s/s/s]')

%% RRR robot Jacobian
%%% H-P Phan
startup_rvc
dbstop if error



clear all syms
syms L_1 L_2 L_3 theta1 theta2 theta3 

L(1) = Link([0 0 0 pi/2], 'sym')
L(1).offset = pi/2;
L(2) = Link([0 L_1 L_2 0], 'sym')
L(2).offset = pi/2;
L(3) = Link([0 0 L_3 0], 'sym')
L(3).offset = 0;

three_link_sym = SerialLink(L, 'name', 'three link symbolic')

digits(5)

J = vpa(three_link_sym.jacob0([theta1 theta2 theta3]));  % Calculate Jacobian

J_matrix =eval(subs(J,  [L_1 L_2 L_3 theta1 theta2 theta3],...
                [1 0.5 0.2  deg2rad([30 60 90])] ))

Jv = J_matrix(1:3, :) %%% Translational Velocity Jacobian 


velocity1 = Jv*[0.5;0.5;0.25] %% Translational Velocity

J1_manipulate =eval(subs(J(1:3, :),  [L_1 L_2 L_3 theta1 theta2 theta3],...
                [1 0.5 0.2 1.0236 1.5472 1.8208] ));
det(J1_manipulate) %% Manipulability at 1(s)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Let try with manual calculation without using the jacob0 function

link1= SerialLink(L(1), 'name', 'Link1');  %% Offset is already set above
link2= SerialLink(L(2), 'name', 'Link2');
link3= SerialLink(L(3), 'name', 'Link3');

OT1=link1.fkine([theta1]).T;    % 0T1
T2=link2.fkine([theta2]).T;     % 1T2
OT2=OT1*T2;                     % 0T2
T3=link3.fkine([theta3]).T;     % 2T3
OT3=OT2*T3;                     % 0T3

%%% the origin of frame {i}: the 4th column of  0Ti
O_0 = [0 0 0]';            
O_1 = OT1(1:3,4);
O_2 = OT2(1:3,4);
O_3 = OT3(1:3,4);

%%% z(i) axis expressed in frame {0}: the 3rd column of 0Ti
Z_0 = [0 0 1]';
Z_1 = OT1(1:3,3);
Z_2 = OT2(1:3,3);
Z_3 = OT3(1:3,3);

%%% Velocity Jacobian RRR robot

Jv_2 = eval(subs([cross(Z_0,(O_3-O_0)) cross(Z_1,(O_3-O_1)) cross(Z_2,(O_3-O_2))],  [L_1 L_2 L_3 theta1 theta2 theta3],...
                [1 0.5 0.2  deg2rad([30 60 90])] ))   

velocity2 = Jv_2*[0.5;0.5;0.25]

%% lspb trajectory

close all; clear variables; clc;
startup_rvc;
dbstop if error
time = 0:0.1:10;
% V= 14; %m/s
qs=10;
qf=80;
t = 10
V = 2 * (qf-qs) / t
figure
[S,SD,SDD] = lspb(qs, qf, time,V);
subplot(3,1,1)
plot(S)
xlim([0 100])
subplot(3,1,2)
plot(SD)
xlim([0 100])
subplot(3,1,3)
plot(SDD)
xlim([0 100])
