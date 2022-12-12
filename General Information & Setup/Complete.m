% Set up
clear all;
startup_rvc;

host = '192.168.59.130';
% host = '192.168.0.100'
port = 30003;
rtde = rtde(host,port);

home = [-588.53, -133.30, 371.91, 2.2214, -2.2214, 0.00];
% Paper bottom left corner
paperBL = [-588.53, -133.30, 0];
% A4 paper size: 210*297
paperTL = paperBL + [297, 210, 0];
% top = 28;
% left = 22; % left side of the paper to left side of the first digit
% length = 297;
% width = 210;
gap = 23; % gap between left side of each digit
% start = [-588.53 - width + top, -133.30 + left, 65, 2.2214, -2.2214, 0.00];

% cases = input("Numbers Plz\n");
% inputPaper = input("Paper position plz\n");  
% angle = input("Rotation angle plz\n");
SequenceDraw(rtde, gap);
% SequenceDraw(rtde, start, gap);
rtde.movel(home);
rtde.close();