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

% Calculate the transformation matrix for 0Tn
% Set n = 6
len = 6;

T = eye(4);
for i = 1:len
    T = T*[cos(thetaB(i)) -sin(thetaB(i))*cos(alpha(i)) sin(thetaB(i))*sin(alpha(i)) a(i)*cos(thetaB(i));
        sin(thetaB(i)) cos(thetaB(i))*cos(alpha(i)) -cos(thetaB(i))*sin(alpha(i)) a(i)*sin(thetaB(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1]
end

