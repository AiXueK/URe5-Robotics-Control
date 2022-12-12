close all;clear variables; clc;
startup_rvc;
dbstop if error
mdl_puma560
host = '192.168.59.130';
% host = '192.168.0.100'
port = 30003;
rtde = rtde(host,port);
home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
% rtde.movel(home);

jointPoseA =  [ -90, -173, 132, 220, 0, 0];
pointC = [100, -127.58, 571.29, -1.571, -0.017, 1.57];
% [poses, jointPose, jointvel, jointaccel] = rtde.movej(jointPoseA, 'joint');
fprintf("move A finished\n")
[posesC, jointPoseC, jointvelC, jointaccelC] = rtde.movel(pointC, 'pose', 1,1,5);
fprintf("move C finished\n")    
rtde.close();