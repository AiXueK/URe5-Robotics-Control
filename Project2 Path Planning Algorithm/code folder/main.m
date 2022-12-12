% Project 2 Main file.
% Use this file to call your project2 function from the project2 file.
% Feel free to modify it as you see fit!

% Declare number of parcels = 3
% totalNumberOfParcels = 4;
totalNumberOfParcels = 5;
% Change pauseTime. Currently set to 0.5
pauseTime = 0.2;

% Desired positions for the parcels
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 0 0 0 0 0];
% [2 0 0 0 0 0];
% [3 0 0 0 0 0];
% ];
% desiredParcels = [
% [0 0 0 0 0 0];
% [0 0 0 0 0 0];
% [0 0 0 0 0 0];
% [1 2 3 0 0 0];
% ];
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 3 0 0 0 0];
% [0 0 0 0 0 0];
% [2 0 0 0 0 0];
% ];

% 4 parcels
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 3 0 0 0 0];
% [2 4 0 0 0 0];
% [0 0 0 0 0 0];
% ];

% 5 parcels
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 5 0 0 0 0];
% [2 4 0 0 0 0];
% [3 0 0 0 0 0];
% ];
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 2 3 4 5 0];
% [0 0 0 0 0 0];
% [0 0 0 0 0 0];
% ];
% desiredParcels = [
% [0 0 0 0 0 0];
% [1 2 0 0 0 0];
% [3 0 0 0 0 0];
% [4 5 0 0 0 0];
% ];
desiredParcels = [
[0 0 0 0 0 0];
[3 0 0 0 0 0];
[1 2 4 0 0 0];
[5 0 0 0 0 0];
];
% Now lets call our algorithm passing in the arguments that we have
% declared above.
sim = project2(desiredParcels,totalNumberOfParcels,pauseTime);

% Get out final positions
finalPositions = sim.positions();


% Now lets display our final positions
disp("The final positions of all of the parcels are:")
disp(finalPositions);
delete(sim);