clear all;
startup_rvc;

% Set up initial variables
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
% Calculate the transformation matrix for 0Tn
% Set n = 6
len = 6;
TA = robot.fkine(thetaA)
TA = double(TA)
TB = robot.fkine(thetaB)
TB = double(TB)
onA = TA((1:3), 4);
onB = TB((1:3), 4);
T = eye(4);
J = [];
zPrev = T((1:3), 3);
oPrev = T((1:3), 4);
for i = 1:len
    T = T*[cos(thetaA(i)) -sin(thetaA(i))*cos(alpha(i)) sin(thetaA(i))*sin(alpha(i)) a(i)*cos(thetaA(i));
        sin(thetaA(i)) cos(thetaA(i))*cos(alpha(i)) -cos(thetaA(i))*sin(alpha(i)) a(i)*sin(thetaA(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1]
    zi = T((1:3), 3);
    oi = T((1:3), 4);
    
    if isempty(J) == 0
        J = [J [cross(zPrev, (onA - oPrev)); zPrev]];
    else
        J = [cross(zPrev, (onA - oPrev)); zPrev];
    end
    zPrev = zi;
    oPrev = oi;
end

