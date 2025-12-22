function R = AngleAxisToRot(h,theta)
% The fuction implement the Rodrigues Formula
% Input: 
% h is the axis of rotation
% theta is the angle of rotation (rad)
% Output:
% R rotation matrix

H = VecToSkew(h);
I = eye(3,3);
R = I + sin(theta)*H + ((1 - cos(theta))*(H^2));

end
