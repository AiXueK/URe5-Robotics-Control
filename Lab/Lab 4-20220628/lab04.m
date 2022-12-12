% Author: Raghav Hariharan
% Lab04 Demonstration code!
% Notes: Run the code one sectiona at a time!
startup_rvc;
%% Example 1
L(1) = Link('revolute', 'd', 0, 'a', 3, 'alpha', 0);  % Link 1. 
L(2) = Link('revolute','d', 0, 'a', 4, 'alpha', 0);  % Link 2.
q1_arm = SerialLink(L, 'name', 'q1 two link');

% Prints out the DH Matrix on the console!
q1_arm

% Uncomment this line open up a GUI to show a manipulateable robotic arm!
% q1_arm.teach 

%% Example 2
% Argument format: [Theta, d, a, alpha]
L(1) = Link([0 0 3 0]);
L(2) = Link([0 0 4 0]);

q2_arm = SerialLink(L, 'name', 'q1 two link');

% Prints out the DH Matrix on the console!
q2_arm

% Uncomment this line open up a GUI to show a manipulateable robotic arm!
% q2_arm.teach 

%% Example 3: Robotic arm with an end effector
% Argument format: [Theta, d, a, alpha]
L(1) = Link([0 0 3 0]);
L(2) = Link([0 0 4 0]);

q3_arm = SerialLink(L, 'name', 'q1 two link');

% Here we are specifying that there is a tool on the robotic arm with 0
% translationl component. 
% This is our third joint.
q3_arm.tool = transl(0, 0, 0);

% Prints out the DH Matrix on the console!
q3_arm

% Uncomment this line open up a GUI to show a manipulateable robotic arm!
% q3_arm.teach 

%% Example 4: SCARA ROBOT RRP

% Order of parameters = [theta, d, a, alpha]
L(1) = Link([0 2 1 pi]);  % Link 1.
L(2) = Link([0 0 1 0]);  % Link 2

% Now lets create the Prismatic link
% Note that we aren't adding the
% "0.5 + d  directly into the link
L(3) = Link([0 0 0 0])  % Link 3


% We then add the 0.5 like so to indicate:
% That rotation is measured from the previous z axis as zero
L(3).offset = 0.5; 

% Then we specify that the joint type is a Prsimatic joint
L(3).jointtype = 'P';

% Finally we specify the range of motion 
% for prismatic joint (must be positive)
% i.e. how far can it extend?
% This means our 'd' variable can extend from 0 to 1
L(3).qlim = [0 1];  

% Put the links together into a serial manipulator
scara_arm = SerialLink(L, 'name', 'Scara');

% Prints the DH Matrix to the console window
scara_arm

theta1 = 0;
theta2 = 0;
d = 0.2;

scara_arm.plot([theta1 theta2 d])

scara_arm.fkine([theta1 theta2 d])



%% Example 5: Forward Kinematics!
% Argument format: [Theta, d, a, alpha]
L(1) = Link([0 0 3 0]);
L(2) = Link([0 0 4 0]);

q2_arm = SerialLink(L, 'name', 'q1 two link');

% Here we are creating our joint configuration
q = [-pi/2, pi/2] 

% Here we calculate the forward kinematic solution, passing in q
q2_arm.fkine(q) 

% We can also plot the robot at this joint configuration like so
q2_arm.plot(q)

%% How do I access the link information about the q2_arm?
% For link 1
q2_arm.links(1).theta
q2_arm.links(1).d
q2_arm.links(1).a
q2_arm.links(1).alpha

% For link 2
q2_arm.links(2).theta
q2_arm.links(2).d
q2_arm.links(2).a
q2_arm.links(2).alpha

