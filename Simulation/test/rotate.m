function [output] = rotate(axis,angle,input)
%H1 Line -- Rotation multiplication relative to the actual frame of
%reference.
%Help Text -- "axis" gives the rotation axis in frame of pre
%frame of reference around which a rotation of "angle" degrees will be
%carried out. v determines if the right multiplicator of the 
%translation matrix; "input" is a vector or a matrix: 
%
% v = 0 => "input" is a vector (only the case for the very first rotation)
% v = 1 => "input" is a matrix
% "output" is always a 4x4 homogeneous transformation matrix

if(axis==1)%  1 means x_axis
    % rotation axis is the actual x axis
    rot = [1 0 0 0; 0 cos(angle) -sin(angle) 0; 0 sin(angle) cos(angle) 0; 0 0 0 1];
end

if(axis==2) % 2 means y_axis
    % rotation axis is the actual y axis
    rot = [cos(angle) 0 sin(angle) 0; 0 1 0 0; -sin(angle) 0 cos(angle) 0; 0 0 0 1];
end

if(axis==3) % 3 means z_axis
    % rotation axis is the actual z axis
    rot = [cos(angle) -sin(angle) 0 0; sin(angle) cos(angle) 0 0; 0 0 1 0; 0 0 0 1];
end

output = rot;