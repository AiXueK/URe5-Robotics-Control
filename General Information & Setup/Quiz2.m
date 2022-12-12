%% Quintic polynomial Trajectories
close all; clear variables; clc;
startup_rvc;
dbstop if error
time = 0:0.1:5;%%% 5 sec

figure
% Start from 0 rad to 1 rad
% velocity limit from 5 to 0
[S,SD,SDD] = tpoly(6/360*pi, 80/360*pi, time,5,0);
%%%%% [S,SD,SDD] = tpoly(0, 1, time); by default the boundaries conditions for velocities are V_i = V_f=0
subplot(3,1,1)

plot(time,S)  %% position trajectory
subplot(3,1,2)
plot(time,SD) %% Velocity
subplot(3,1,3)
plot(time,SDD) %Acceleration
%% Q2
%% RRR robot Jacobian
%%% H-P Phan
startup_rvc
dbstop if error



clear all syms
syms L_1 L_2 L_3 theta1 theta2 theta3 

L(1) = Link([0 0.4 0.8602 90/360*pi], 'sym')
L(2) = Link([0 0.2 0 0], 'sym')

two_link_sym = SerialLink(L, 'name', 'two link symbolic')

digits(5)

J = vpa(two_link_sym.jacob0([20/360*pi 0]));  % Calculate Jacobian

% J_matrix =eval(subs(J,  [L_1 L_2 theta1 theta2 theta3],...
%                 [1 0.5 0.2  deg2rad([30 60 90])] ))

Jv = J(1:3, :) %%% Translational Velocity Jacobian 


% velocity1 = Jv*[0.5;0.5;0.25] %% Translational Velocity

% J1_manipulate =eval(subs(J(1:3, :),  [L_1 L_2 L_3 theta1 theta2 theta3],...
%                 [1 0.5 0.2 1.0236 1.5472 1.8208] ));
% det(J1_manipulate) %% Manipulability at 1(s)
%%
startup_rvc;

% Calculate the revolute link transformation
thetaDeg = [20 0];
d = [0.4 0.2];
a = [0.8602 0];
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
% q_dot = inv(Jxy)*[0.1 0]'
%%
%% lspb trajectory

close all; clear variables; clc;
startup_rvc;
dbstop if error
time = 0:0.1:10;
% V= 14; %m/s
qs=0;
qf=40;
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

%%
-1*5*([5;0] - [0;5])/norm([5;0] - [0;5]);
-1*4*([5;7] - [-7;5])/norm([5;7] - [-7;5]);
ro1 = 0.8;
grad1 = [(5-5.8)/ro1;0];
7*sqrt(1/ro1 - 1)*1/ro1^2 * grad1;
ro2 = sqrt((5-5.3)^2 + (7 - 7)^2);
grad2 = [(5-5.3)/ro2;0];
8*sqrt(1/ro2 - 1)*1/ro2^2 * grad2
%%
%% Use the Joint Positions to sub in and calculate the actual Translational and Rotation Velocity Jacobian
L(1) = Link([1 0.6 0 -55/360*pi]);
one_link_COG = SerialLink(L(1), 'name', 'one link');

% Here we create a 2 link robotic arm, but instead of using ac1 for the
% first link we use a1. This is because link 2 supports the entire weight
% of link1, so the entire lenght of the link must be considered.

% L(1) = Link([0 0 a1 0]);
L(2) = Link([1 0.4 0 0]);
two_link_COG = SerialLink(L, 'name', 'two link');


digits(5)

J = vpa(two_link_COG.jacob0([0 0]));  % Calculate Jacobian

% J_matrix =eval(subs(J,  [L_1 L_2 L_3 theta1 theta2 theta3],...
%                 [1 0.5 0.2  deg2rad([30 60 90])] ))

Jv1 = J(1:3, :); %%% Translational Velocity Jacobian 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%Let try with manual calculation without using the jacob0 function

link1= SerialLink(L(1), 'name', 'Link1');  %% Offset is already set above
link2= SerialLink(L(2), 'name', 'Link2');

OT1=link1.fkine([0]).T;    % 0T1
T2=link2.fkine([0]).T;     % 1T2
OT2=OT1*T2;                     % 0T2
             % 0T3

%%% the origin of frame {i}: the 4th column of  0Ti
O_0 = [0 0 0]';            
O_1 = OT1(1:3,4);
O_2 = OT2(1:3,4);

%%% z(i) axis expressed in frame {0}: the 3rd column of 0Ti
Z_0 = [0 0 1]';
Z_1 = OT1(1:3,3);
Z_2 = OT2(1:3,3);

%%% Velocity Jacobian RRR robot

Jv2 = [Z_0, Z_1];
% Jv_2 = eval(subs([cross(Z_0,(O_3-O_0)) cross(Z_1,(O_3-O_1))],  [L_1 L_2 theta1 theta2],...
%                 [1 0.5 0.2  deg2rad([30 60 90])] ))   
% 
% velocity2 = Jv_2*[0.5;0.5;0.25]

q = jointPositions';
q_dot = jointVelocities';
q_ddot = jointAcceleration';

% Calculation of Inertia: Izz = m/12(a^2+b^2)
% a and b here are used to matchup with the syntax of the Izz calculation
% presented on lecture slide 25
m1 = 2;
a1 = 0.1; % This is the width of link 1 
b1 = 0.8; % This is the length a of the link 1

m2 = 2;
a2 = 0.1; % This is the width of link 2
b2 = 0.6; % This is the length a of the link 2

% Here are the values for Inertia!
Izz_1 = m1/12*(a1^2 +b1^2)
Izz_2 = m2/12*(a2^2 +b2^2)

% ----Rotational kinetic energy-----
% In the lecture slides you will see a variable Ri:
% Ri should be the rotation matrix obtained from the forward kinematic for frame (i) with respect to frame (0). 
% However, as all rotational matrix are parallel to zo, then we can just simplify it as Izz
% Basically all the motors are rotating around the same axis.

Kr = 1/2*q_dot'*(Jw1'*Izz_1*Jw1 + Jw2'*Izz_2*Jw2)*q_dot;

% ---- Translation Kinetic Energy -----

Kt = 1/2*q_dot'*(m1*(Jvc1)'*Jvc1 + m2*(Jvc2)'*Jvc2)*q_dot;

% Kinetic energy = Rotational Kinetic energy + translation kinetic energy
K = Kr + Kt




% ----------------------------------------------------------------
% Step 5: Construct the manipulator matrix D(q)
% Follow the equation on slide 48
d = m1*(Jvc1)'*Jvc1 + m2*(Jvc2)'*Jvc2 + [[(Izz_1+Izz_2) Izz_1];[Izz_2 Izz_2]]