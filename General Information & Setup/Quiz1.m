%% Q6
B = [0 0 0 1];
step1 = [[rotz(125,'deg') [0;0;0]];B]
step2 = eye(4);
step2(2,4) = 0.2
step3 = eye(4);
step3(1,4) = 0.3
step4 = [[roty(60,'deg') [0;0;0]; B]]
result = step1*step2*step3*step4

%% Q7
startup_rvc;

% Calculate the revolute link transformation
thetaDeg = [35 0 0];
d = [13 19 14];
a = [0 0 0];
alphaDeg = [0 -90 0];
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

%% Q3
F = 8*10^6*pi*0.05^2
Q = 0.03*pi*0.05^2*10^6

%% Q4
Vout = 1/4*2/1740
Vout = Vout*10^3
F = 0.9/Vout*1

%% Q5
step1 = rotx(55, 'deg')
result = roty(50,'deg')*rotz(45,'deg')*rotx(55,'deg')*rotz(110,'deg')*roty(40,'deg')