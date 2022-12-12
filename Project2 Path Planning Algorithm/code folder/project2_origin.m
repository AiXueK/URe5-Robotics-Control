% DO NOT CHANGE THE FILE NAME OR IT WILL NOT WORK DURING TESTING
% Author: 
% zid:
% Date:


% DO NOT CHANGE THE FUNCTION DEFINITION. OR IT WILL FAIL THE TESTS
% You are welcome to call other functions from this main function.
function sim = project2(desiredPosition, totalNumberOfParcesl, pauseTime)
    % This line needs to be the very first line. Do not remove it.
    sim = project2simulator(totalNumberOfParcesl,pauseTime);


    % TODO  
    % Call another function here.
    % The following move function is an example. Feel free to replace it.
    move(sim);
    
end

% This is just an example function to show the functionality of the
% project2simulator
function move(sim)
    % Parcel 1 is at the top of box 1

    % Pick up from box 1
    % This function automatically picks up whatever is at the top of the
    % stack. As parcel 1 is at the top, it will be picked up.
    sim.pickup(1)

    % Put down what we have picked up 
    sim.putdown(2)

    % Print the position of piece 1.
    % It should say 2. Indicating that parcel 1 is now in box 2.
    position = sim.currentPos(1);
    disp(position)

    % What if we tried to putdown a parcel without having picked up
    % anything? It will return a warning saying you need to pick something
    % up. "The vacuum gripper is not holding anything. Please pickup something first Box 3 is empty "
    sim.putdown(2)

    % What if we tried to pick up a parcel from an empty box? It will
    % return a warning saying that the box is empty. "Box 3 is empty"
    sim.pickup(3)

    % What if we tried to pick up twice in sucession? A warning saying
    % "Vacuum gripper is holding something. Please put it down first" will
    % pop up.
    sim.pickup(1)
    sim.pickup(1)
    sim.putdown(3)

    % I can also print out the positions of the boxes like this to read it
    % in a more human like fashion
    sim.printPositions();

    % I can also do it this way. To show a matrix
    sim.positions()


    % What happens if we try to place a larger piece ontop of a smaller
    % piece? A warning saying "!!!!!!!!!MOVEMENT NOT VALID!!!!!!!!!" popsup
    % This will invalidate the entire program
    sim.pickup(1)
    sim.putdown(3)

    % If i print out the matrix of the position once again. I can see that
    % all of the 0's have now changed to "-1". This means that your
    % solution has failed the condition of always placing the smaller
    % parcel ontop of a larger parcel.
    sim.positions()
    
    % The violation will still persist even if you continue the program.
    sim.pickup(3)
    sim.putdown(4)

end