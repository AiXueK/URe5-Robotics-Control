% DO NOT CHANGE THE FILE NAME OR IT WILL NOT WORK DURING TESTING
% Author: Ruiqi Xie 
% zid:z5141973
% Date:


% DO NOT CHANGE THE FUNCTION DEFINITION. OR IT WILL FAIL THE TESTS
% You are welcome to call other functions from this main function.
function sim = project2(desiredPosition, totalNumberOfParcesl, pauseTime)
    % This line needs to be the very first line. Do not remove it.
    sim = project2simulator(totalNumberOfParcesl,pauseTime);


    % TODO  
    % Call another function here.
    % The following move function is an example. Feel free to replace it.
    move(desiredPosition, sim);
    
end

% This is just an example function to show the functionality of the
% project2simulator
function move(desiredPosition, sim)
    global spare;
    destination = getDestination(desiredPosition);
    nList = getnList(destination);
    % TODO: already got the destination and n values; Need to write hanoi
    % functions based on these values
    total = find(destination > 0);
    start = 1;
    spare = [0, 0, 0, 0];
    for i = total
        if sim.currentPos(i) == destination
            continue
        end
        h(nList(i), start, destination(i), sim);
    end
end


function h(goal, start, des, sim)
    hanoi(sim, goal, start, des);
end

function hanoi(sim, num, start, des)
    global spare;
    if num == 1
        sim.pickup(start)
        sim.putdown(des)
        spare(start) = 1;
        spare(des) = 1;
        return
    else
        if spare(des) == 0
            hanoi(sim, num-1, start, des);
        else
            available = find(spare == 0);
            hanoi(sim, num-1, start, available(1));
            hanoi(sim, 1, start, des);
            hanoi(sim, num-1, available(1), des)
        end        
    end
end

% function checkAvailable(sim, parcel, )

% Get a list of destination box of the parcels
function destination = getDestination(desiredPosition)
    destination = [0, 0, 0, 0, 0, 0];
    for parcel = 1:5
        [row, col] = find(desiredPosition == parcel);
        if isempty(row)
            break
        end
        destination(parcel) = row;
    end
end

% Get nList that stores the n for hanoi function
function nList = getnList(destination)
    nList = [];
    for i = 1:(length(destination) - 1)
        n = 1;
        if destination(i + 1) == 0
            break;
        elseif destination(i) ~= destination(i + 1)
            nList = [nList;n];
            n = 1;
        else
            n = n + 1;
        end
    end
    nList = [nList;n];
end