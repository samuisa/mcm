function [h, theta] = RotToAngleAxis(R)
% RotToAngleAxis: converts a rotation matrix into an angle-axis representation.
% Input: R - 3x3 rotation matrix
% Output: h - rotation axis (3x1 unit vector), theta - rotation angle (in rad)

    % Verifica validità matrice di rotazione
    if ~IsRotationMatrix(R)
        warning('Not valid rotation matrix');
        h = [NaN; NaN; NaN];
        theta = NaN;
        return;
    else
        % Computing angle of rotation 
    theta = acos((trace(R) - 1) / 2);

    % Case angle=0
    if abs(theta) < 1e-3
        h = [1; 0; 0]; % arbitrary
        return;
    end

    % Case angle=pi
    if abs(theta - pi) < 1e-3
        a = diag(R);
        for i = 1:length(a)
            h_dagger = abs(sqrt((a(i)+1)/2));
            if h_dagger > 0
                if i == 1
                    h = [h_dagger;
                        sign(h_dagger)*sign(R(1,2))*abs(sqrt((a(2)+1)/2));
                        sign(h_dagger)*sign(R(1,3))*abs(sqrt((a(3)+1)/2))];
                    h = h / norm(h);
                    return;
                elseif i == 2
                    h = [sign(h_dagger)*sign(R(2,1))*abs(sqrt((a(1)+1)/2));
                        h_dagger;
                        sign(h_dagger)*sign(R(2,3))*abs(sqrt((a(3)+1)/2))];
                    h = h / norm(h);
                    return;
                elseif i == 3
                    h = [sign(h_dagger)*sign(R(3,1))*abs(sqrt((a(1)+1)/2));
                        sign(h_dagger)*sign(R(3,2))*abs(sqrt((a(2)+1)/2));
                        h_dagger];
                    h = h / norm(h);
                    return;
                end
            end
        end
    end

    % General case
    h = vex((R - R') / 2) / sin(theta);
    h = h / norm(h);

    end
end


function a = vex(S)
% VEX: converts a 3x3 skew-symmetric matrix into a 3x1 vector
    a = [S(3,2); S(1,3); S(2,1)];
end
