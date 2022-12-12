% DO NOT CHANGE THE FILE NAME OR IT WILL NOT WORK DURING TESTING

% stack calculate

% DO NOT CHANGE THE FUNCTION DEFINITION. OR IT WILL FAIL THE TESTS
% You are welcome to call other functions from this main function.
function sim = project2(desiredPosition, totalNumberOfParcesl, pauseTime)
    % This line needs to be the very first line. Do not remove it.
    sim = project2simulator(totalNumberOfParcesl,pauseTime);


    % Call another function here.
    % The following move function is an example. Feel free to replace it.
    move(desiredPosition, sim);
    
end

% This is just an example function to show the functionality of the
% project2simulator
function move(desiredPosition, sim)
    global spare destination parcel;
    destination = getDestination(desiredPosition);
    total = find(destination > 0);
    start = 1;
    sizeDes = size(destination);
    spare = zeros(1, sizeDes(1));
    % Loop through the parcels to put them to desired positions
    for i = total
        parcel = length(total) + 1 - i;
        if sim.currentPos(parcel) == destination(parcel)
            continue
        end
        % Changed
        put(sim, parcel, start, destination(parcel));
    end
end

% Remove parcels above and move gaol parcel to desired position
function put(sim, parcel, start, des)
    global spare destination;
    parcelBox = sim.currentPos(parcel);
    [row, col] = find(sim.positions == parcel);
    n = col - 1;
    if (n == 0)
        hanoi(sim, 1, parcelBox, des);
        return
    end
    checkAvailable(sim, n);
    spare(des) = 1;
    if spare(destination(n)) == 0
        available = destination(n);
    else
        available = find(spare == 0);
        available(available == start) = [];
    end
    % Implement the move operation
    hanoi(sim, n, parcelBox, available(1));
    hanoi(sim, 1, parcelBox, des);
end

% Recursively move the stack of parcels to destination
function hanoi(sim, num, start, des)
    global spare parcel destination;
    position = sim.positions();
    current = position(start, 1);
    checkAvailable(sim, current);
    spare(start) = 1;
    spare(des) = 1;
    if num == 1
        sim.pickup(start)
        sim.putdown(des)
        spare(start) = 0;
        return
    else
        check = sim.currentPos(parcel);
        if ((position(check, 1) == parcel) && position(destination(parcel), 1) == 0)
            return;
        end
        if spare(des) == 0
            available = des;
        else
            available = find(spare == 0);
            available(available == start) = [];
        end       
        hanoi(sim, num-1, start, available(1));
        position = sim.positions();
        if ((position(check, 1) == parcel) && position(destination(parcel), 1) == 0)
            return;
        end
        hanoi(sim, 1, start, des);
        position = sim.positions();
        if ((position(check, 1) == parcel) && position(destination(parcel), 1) == 0)
            return;
        end
        hanoi(sim, num-1, available(1), des)
    end
end

% Check the spared boxes
function checkAvailable(sim, current)
    global spare;
    position = sim.positions();
    sizeDes = size(position);
    for i = 2:sizeDes(1)
        if position(i, 1) <= current && position(i, 1) ~= 0
            spare(i) = 1;
        else
            spare(i) = 0;
        end
    end
end

% Get a list of destination box of the parcels
function destination = getDestination(desiredPosition)
    sizeDes = size(desiredPosition);
    destination = zeros(1, sizeDes(2));
    for parcel = 1:5
        [row, col] = find(desiredPosition == parcel);
        if isempty(row)
            break
        end
        destination(parcel) = row;
    end
end