% Author: Raghav Hariharan
% For lab09
% startup_rvc;
clear all

%% In this section we are calculating the Z & O matrices given the actual link sizes.

syms  a1 a2 ac1 ac2 theta1 theta2

% Uncomment lines 11 - 16 when you want to use the real values
a1 = 0.8;
a2 = 0.6;
ac1 = a1/2;
ac2 = a2/2;
theta1 = 0.520980781720307	 % This was calculated from a subsequent section
theta2 = 0.781471172580461 % This was calculate from a subsequent section

L(1) = Link([0 0 a1 0]);
L(2) = Link([0 0 a2 0]);
one_link = SerialLink(L(1), 'name', 'one link');
two_link = SerialLink(L, 'name', 'two link');


OT1 = one_link.fkine([theta1]).T;
OT2 = two_link.fkine([theta1 theta2]).T;

% Calculate out O and Z matrices:
O_0 = [0 0 0]';
O_1 = OT1(1:3,4);
O_2 = OT2(1:3,4);

% This is the most important part.
Z_0 = [0 0 1]';
Z_1 = OT1(1:3,3);
Z_2 = OT2(1:3,3);


%% Here we use the COG of each link instead to calculate the O_cog matrices
% NOTE: This section only returns uses the symbloic parameters to return the
% equations.
% Now we can calculate the matricies from the COG.
% O_c :  is the position vector of the centre of mass of the ith link
% ac1 = a1/2 
% ac2 = a2/2

L(1) = Link([0 0 ac1 0]);
one_link_COG = SerialLink(L(1), 'name', 'one link');

% Here we create a 2 link robotic arm, but instead of using ac1 for the
% first link we use a1. This is because link 2 supports the entire weight
% of link1, so the entire lenght of the link must be considered.

L(1) = Link([0 0 a1 0]);
L(2) = Link([0 0 ac2 0]);
two_link_COG = SerialLink(L, 'name', 'two link');

% Calculate the forward kinematic solution of each link
OT1cog = one_link_COG.fkine([theta1]).T;
OT2cog = two_link_COG.fkine([theta1 theta2]).T;


% Calculate out O_cog matrices:
O_0cog = [0 0 0]';
O_1cog = OT1cog(1:3,4);
O_2cog = OT2cog(1:3,4);


%% Calculate the Translational velocity Jacobian
% We have now calculated the Z0 - Z2 and O_0 - O2 and O_0cog - O_2cog
% These equations should match with what was presented in the lectures!
Jvc1 = [cross(Z_0,(O_1cog-O_0)) [0 0 0]']
Jvc2 = [cross(Z_0,(O_2cog-O_0)) cross(Z_1,(O_2cog-O_1))]

Jw1 = [Z_0 [0 0 0]']
Jw2 = [Z_0 Z_1]

 
%%  Question 2: Generate the Trapezoidal trajectory
L(1) = Link([0 0 0.8 0]);
L(2) = Link([0 0 0.6 0]);
two_link_real = SerialLink(L, 'name', 'two link');

t = [0:0.01:6];
s = mtraj(@lspb, [0 0], deg2rad([60 90]), t);
sd = [zeros(1,2); diff(s)];
sdd = [zeros(1,2); diff(sd)];
sddd = [zeros(1,2); diff(sdd)];

%Show position, velocity and acceleration as a function of time
f1 = figure(1);
subplot(4,1,1)
plot(t, s); grid on; xlim([min(t), max(t)]);
title('Trapezoidal Trajectory');
xlabel('Time');
ylabel('Position [rad]')
legend('q1', 'q2');
subplot(4,1,2)
plot(t, sd); grid on; xlim([min(t), max(t)]);
xlabel('Time'); 
ylabel('Velocity [rad/s]')
subplot(4,1,3)
plot(t, sdd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Acceleration [rad/s/s]')
subplot(4,1,4)
plot(t, sddd); grid on; xlim([min(t), max(t)]);
xlabel('Time');
ylabel('Jerk [rad/s/s/s]')


f2 = figure(2);
T = two_link_real.fkine(s);
p = transl(T);  % Just translational components
plot(p(:, 1), p(:, 2)); axis equal; xlabel('X [m]'); ylabel('Y [m]'); grid on; title('Path');

% Uncomment this line to see the robot moving.
two_link_real.plot(s)

% Angular velocity and accleration at time = 3s

jointPositions = s(300,:) % rads
jointVelocities = sd(300,:) % rads/s
jointAcceleration = sdd(300,:) % rads/s^2 (It is pretty much 0)

%% Use the Joint Positions to sub in and calculate the actual Translational and Rotation Velocity Jacobian
q = jointPositions;
q_dot = jointVelocities;
q_ddot = jointAcceleration;

a1 = 0.8;
a2 = 0.6;
ac1 = a1/2;
ac2 = a2/2;
theta1 = q(1);
theta2 = q(2); 

L(1) = Link([0 0 ac1 0]);
one_link_COG = SerialLink(L(1), 'name', 'one link');

% Here we create a 2 link robotic arm, but instead of using ac1 for the
% first link we use a1. This is because link 2 supports the entire weight
% of link1, so the entire lenght of the link must be considered.

L(1) = Link([0 0 a1 0]);
L(2) = Link([0 0 ac2 0]);
two_link_COG = SerialLink(L, 'name', 'two link');

% Calculate the forward kinematic solution of each link
OT1cog = one_link_COG.fkine([theta1]).T;
OT2cog = two_link_COG.fkine([theta1 theta2]).T;


% Calculate out O_cog matrices:
O_0cog = [0 0 0]';
O_1cog = OT1cog(1:3,4);
O_2cog = OT2cog(1:3,4);


%% Calculate the Translational velocity Jacobian
% We have now calculated the Z0 - Z2 and O_0 - O2 and O_0cog - O_2cog
% These equations should match with what was presented in the lectures!
Jvc1 = [cross(Z_0,(O_1cog-O_0)) [0 0 0]']
Jvc2 = [cross(Z_1,(O_2cog-O_0)) cross(Z_0,(O_2cog-O_1))]

Jw1 = [Z_0 [0 0 0]']
Jw2 = [Z_0 Z_1]

%% Question 3: Calculate the Torques applied at each joint at t = 3 seconds.
% From the powerpoint slide 46
% Step 4: Kinetic Energy
q = jointPositions';
q_dot = jointVelocities';
q_ddot = jointAcceleration';

% Calculation of Inertia: Izz = m/12(a^2+b^2)
% a and b here are used to matchup with the syntax of the Izz calculation
% presented on lecture slide 25
m1 = 4;
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





% ----------------------------------------------------------------
% Step 6: Apply the Euler-Larange Equations
% Calculation of Coriolis and centripetal coupling matrix
% (slide 50)
c111 = 0;
h = -m2*a1*ac2*sin(theta2);
c121 = h
c211 = c121;
c221 = h;
c112 = -h;
c122 = 0
c212 = c122;
c222 = 0;

% Calculation of gravity loading 
% Slide 51
g = 9.81;
g1 = (m1*ac1 + m2*a2)*g*cos(theta1) + m2*ac2*g*cos(theta1 + theta2)
g2 = m2*ac2*g*cos(theta1 + theta2);

% Joint 1 Torque!:
t1 = d(1,1)*q_ddot(1) + d(1,2)*q_ddot(2) + c121*q_dot(1)*q_dot(2) + c211*q_dot(2)*q_dot(1) + c221*q_dot(2)^2 + g1

% Joint 2 Torque!:
t2 = d(2,1)*q_ddot(1) + d(2,2)*q_ddot(2) + c112*q_dot(1)^2 + g2

% You can see that the torque on joint 1 is much higher as it needs to
% support itself as well as joint 2!
