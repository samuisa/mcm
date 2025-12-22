function [h, theta] = RotToAngleAxisEig(R)

    [vectors, values] = eigs(R);
    values = real(values);
    vectors = real(vectors);
    for i = 1:3
        if norm(values(i,i) - 1) < 1e-3
           h = vectors(:, i);
         end
    end
    h = h/norm(h);
    theta = acos((trace(R) - 1) / 2);

    %H = VecToSkew(h);
    %cos_theta = (trace(R) - 1) / 2;
    %sin_theta = -trace(H * R) / 2;
    %theta = atan2(sin_theta, cos_theta); %  calculating theta with both sin and cos components
    
end