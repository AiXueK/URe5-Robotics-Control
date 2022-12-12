close all;clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560
host = '192.168.59.130';
% host = '192.168.0.100'
port = 30003;
rtde = rtde(host,port);
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
rtde.movel(home);

point1 = [-588.53, -133.30, 100, 2.2214, -2.2214, 0.00];
point2 = [-688.53, -133.30, 100, 2.2214, -2.2214, 0.00];
point3 = [-688.53, -233.30, 100, 2.2214, -2.2214, 0.00];
point4 = [-588.53, -233.30, 100, 2.2214, -2.2214, 0.00];
% acc = 0.1;
% v = 0.05;
rtde.movel(point1,'pose', 0.1, 0.05)
% [poses1, jointpos1, jointvel1, jointaccel1] = rtde.movel(point1, 'pose', 0.1, 0.05);
[poses2, jointpos2, jointvel2, jointaccel2] = rtde.movel(point2, 'pose', 0.1, 0.05);
[poses3, jointpos3, jointvel3, jointaccel3] = rtde.movel(point3, 'pose', 0.1, 0.05);
[poses4, jointpos4, jointvel4, jointaccel4] = rtde.movel(point4, 'pose', 0.1, 0.05);
[poses5, jointpos5, jointvel5, jointaccel5] = rtde.movel(point1, 'pose', 0.1, 0.05);

posesl = [poses2;poses3;poses4;poses5];
jointposel = [jointpos2;jointpos3;jointpos4;jointpos5];
jointvell = [jointvel2;jointvel3;jointvel4;jointvel5];
jointaccell = [jointaccel2; jointaccel3; jointaccel4;jointaccel5];


rtde.movel(home);
rtde.movej(point1,'pose', 0.1, 0.05)
% [poses1, jointpos1, jointvel1, jointaccel1] = rtde.movej(point1, 'pose', 0.1, 0.05);
[poses2, jointpos2, jointvel2, jointaccel2] = rtde.movej(point2, 'pose', 0.1, 0.05);
[poses3, jointpos3, jointvel3, jointaccel3] = rtde.movej(point3, 'pose', 0.1, 0.05);
[poses4, jointpos4, jointvel4, jointaccel4] = rtde.movej(point4, 'pose', 0.1, 0.05);
[poses5, jointpos5, jointvel5, jointaccel5] = rtde.movej(point1, 'pose', 0.1, 0.05);

posesj = [poses2;poses3;poses4;poses5];
jointposej = [jointpos2;jointpos3;jointpos4;jointpos5];
jointvelj = [jointvel2;jointvel3;jointvel4;jointvel5];
jointaccelj = [jointaccel2; jointaccel3; jointaccel4;jointaccel5];


rtde.movel(home);
rtde.close();
%%
xtransl = posesl(:, 1);
ytransl = posesl(:, 2);
ztransl = posesl(:, 3);
xtransj = posesj(:, 1);
ytransj = posesj(:, 2);
ztransj = posesj(:, 3);

figure(1)
plot(xtransl)
hold on
plot(xtransj)
legend("movel", "movej")
title("X translational component")
ylabel("position (m)")
xlabel("time (s)")
hold off
figure(2)
plot(ytransl)
hold on
plot(ytransj)
legend("movel", "movej")
title("Y translational component")
ylabel("position (m)")
xlabel("time (s)")
hold off
figure(3)
plot(ztransl)
hold on
plot(ztransj)
legend("movel", "movej")
title("Z translational component")
ylabel("position (m)")
xlabel("time (s)")
hold off

drawPath(posesl, 'TCP path movel');
drawPath(posesj, 'TCP path movej');

drawJointPositions(jointposel, "Joint Positions movel")
drawJointPositions(jointposej, "Joint Positions movej")

drawJointVelocities(jointvell, "radians/s", 'Joint Velocities movel')
drawJointVelocities(jointvelj, "radians/s", 'Joint Velocities movej');

drawJointAccelerations(jointaccell, "radians/s^2", 'Joint Accelerations movel');
drawJointAccelerations(jointaccelj, "radians/s^2", 'Joint Accelerations movel');

