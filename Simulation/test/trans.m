function [output] = trans(x,y,z)%,input)    %trans(x,y,z,v,input)
%H1 Line -- Translation multiplication relative to the actual frame of
%reference.
%Help Text -- x,y,z, give the 3-tuple translation vector in frame of pre
%frame of reference. v determines if the right multiplicator of the 
%translation matrix; "input" is a vector or a matrix: 
%
% v = 0 => "input" and also "output" is a vector
% v = 1 => "input" and also "output" is a matrix

%if(v==vector)
%    output = zeros(1,3);
%    
%    output(1) = input(1) + x;
%    output(2) = input(2) + y;
%    output(3) = input(3) + z;
%end

%if(v==matrix)
    %output = zeros(4,4);
    
    %output = input;
    output = eye(4);
    output(1,4) = output(1,4) + x;
    output(2,4) = output(2,4) + y;
    output(3,4) = output(3,4) + z;
%end