function phi = so3Log(R)
    theta = acos( max(min((trace(R)-1)/2,1),-1) );

    if theta < 1e-6
        phi = zeros(3,1);
    else
        phi = theta/(2*sin(theta)) * ...
              [ R(3,2)-R(2,3);
                R(1,3)-R(3,1);
                R(2,1)-R(1,2) ];
    end
end
