% Set up
clear all;
startup_rvc;
% Initial variables set up
jointA = [-90 -173 132 220 0 0];
jointB = [-90 -60 90 0 90 0];
d = [0.1625 0 0 0.1333 0.0997 0.0996];
a = [0 -0.425 -0.3922 0 0 0];
alpha = [pi/2 0 0 pi/2 -pi/2 0];
thetaA = deg2rad(jointA);
thetaB = deg2rad(jointB);

len = 6;

for i = 1:len
    L(i) = Link('revolute', 'd', d(i), 'a', a(i), 'alpha', alpha(i), 'offset', 0);
end
robot = SerialLink(L, 'name', 'two link');
TA = robot.fkine(thetaA);
TB = robot.fkine(thetaB);
% Find RPY values for transfomation matrices
% rpyA = tr2rpy(TA)
% rpyB = tr2rpy(TB)

% Calculate Jacobian at configuration qi
JA = robot.jacob0(thetaA);
JB = robot.jacob0(thetaB)

Jxy = JA(1:2, :);
q_dotA = pinv(Jxy)*[0.25 0]';

q_dotB = [1, 0.1, 0.1, 0.1, 0.1, 0.1]'
d = JB * q_dotB
