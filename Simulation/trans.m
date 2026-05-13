function [output] = trans(x,y,z)
%H1 Line -- Translation multiplication relative to the actual frame of
%reference.
%Help Text -- x,y,z, give the 3-tuple translation vector in frame of pre
%frame of reference. v determines if the right multiplicator of the 
%translation matrix: 

    output = eye(4);
    output(1,4) = output(1,4) + x;
    output(2,4) = output(2,4) + y;
    output(3,4) = output(3,4) + z;
