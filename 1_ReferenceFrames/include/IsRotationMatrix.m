function [isRotationMatrix] = IsRotationMatrix(R)
% The function checks that the input R is a valid rotation matrix, that is 
% a valid element of SO(3).
% Return true if R is a valid rotation matrix, false otherwise. In the
% latter case, print a warning pointing out the failed check.

isOrthogonal = norm(R*R' - eye(3)) < 1e-3;
isProper = abs(det(R) - 1) < 1e-3;
isRotationMatrix = isOrthogonal && isProper;

end