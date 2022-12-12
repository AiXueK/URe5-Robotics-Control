clc;
clear all;
desiredParcels = [
[0 0 0 0 0 0];
[1 0 0 0 0 0];
[2 0 0 0 0 0];
[3 0 0 0 0 0];
];
% TCP Host and Port settings
% host = '192.168.59.130';
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
rtdeport = 30003;
vacuumport = 63352;

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,rtdeport);

% Calling the constructor of vacuum to setup tcp connction
vacuum = vacuum(host,vacuumport);

home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
rtde.movej(home);

n = 5;
h = 3;
safe = 0;
box1 = [-700, -100, 0, 2.2214, -2.2214, 0.00];
box2 = [-500, -100, 0, 2.2214, -2.2214, 0.00];
box3 = [-700, -300, 0, 2.2214, -2.2214, 0.00];
box4 = [-500, -300, 0, 2.2214, -2.2214, 0.00];

totalPose = [];
% Execution part
% pick up parcel1 from box1
pose1 = rtde.movej(box1 + [0 0 23 0 0 0]);
vacuum.grip()
% put parcel 1 down to box2
pose2 = rtde.movel(box1 + [0 0 27 0 0 0]);
pose3 = rtde.movel(box2 + [0 0 27 0 0 0]);
pose4 = rtde.movel(box2 + [0 0 15 0 0 0]);
vacuum.release()
pause(0.2)
% go back to box1 and pick up parcel 2
pose5 = rtde.movel(box2 + [0 0 22 0 0 0]);
pose6 = rtde.movel(box1 + [0 0 22 0 0 0]);
pose7 = rtde.movel(box1 + [0 0 21 0 0 0]);
vacuum.grip()
% go to box3 and put down parcel 2
pose8 = rtde.movel(box1 + [0 0 25 0 0 0])
pose9 = rtde.movel(box3 + [0 0 25 0 0 0]);
pose10 = rtde.movel(box3 + [0 0 15 0 0 0]);
vacuum.release()
pause(0.2)
% go to box1 and pick up parcel 3
rtde.movel(box1 + [0 0 25 0 0 0]);
pose11 = rtde.movel(box1 + [0 0 18 0 0 0]);
vacuum.grip()
% go to box4 and put down parcel 4
rtde.movel(box1 + [0 0 25 0 0 0]);
pose12 = rtde.movel(box4 + [0 0 h+safe 0 0 0]);
vacuum.release()
pause(0.2)

totalPose = [pose1;pose2;pose3;pose4;pose5;pose6;pose7;pose8;pose9;pose10;
    pose11;pose12];

rtde.movej(home);

% and plot the path of the TCP
rtde.drawPath(totalPose);

% Closing the TCP Connection
rtde.close();