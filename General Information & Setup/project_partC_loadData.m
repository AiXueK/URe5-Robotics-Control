close all;clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560

load("posesl.mat", "posesl");
load("posesj.mat", "posesj");
load("jointposel.mat", "jointposel");
load("jointposej.mat", "jointposej");
load("jointvell.mat", "jointvell");
load("jointvelj.mat", "jointvelj");
load("jointaccell.mat", "jointaccell");
load("jointaccelj.mat", "jointaccelj");
% 96 movel; 232 movej
% index = find(posesl == [-0.5886   -0.1333    0.1001    2.2213   -2.2213   -0.0002])

posesl = posesl(96:end, :);
posesj = posesj(232:end,:);
jointposel = jointposel(96:end,:);
jointposej = jointposej(232:end,:);
jointvell = jointvell(96:end,:);
jointvelj = jointvelj(232:end,:);
jointaccell = jointaccell(96:end,:);
jointaccelj = jointaccelj(232:end,:);

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
drawJointAccelerations(jointaccelj, "radians/s^2", 'Joint Accelerations movej');

