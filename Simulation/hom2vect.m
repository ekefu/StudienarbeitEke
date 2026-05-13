function [vect] = hom2vect(matrix_hom)
% Converts the translational part of a homogeneous transformation matrix
% into a 3D vector for cartesian space visualisations.


if((size(matrix_hom,1) == 4)&&(size(matrix_hom,1) == 4))

% vect = zeros(1,3);
% 
% vect(1,1) = matrix_hom(1,4);
% vect(1,2) = matrix_hom(2,4);
% vect(1,3) = matrix_hom(3,4);

vect = zeros(3,1);

vect(1,1) = matrix_hom(1,4);
vect(2,1) = matrix_hom(2,4);
vect(3,1) = matrix_hom(3,4);


else
    error('Input to the function hom2vect must be a 4x4 matrix.')
end