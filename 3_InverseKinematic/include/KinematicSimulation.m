function [q_new] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
% KinematicSimulation
% Integrates joint velocities and applies joint limits
%
% Inputs:
% qi     : current joint configuration (nx1)
% q_dot  : joint velocities (nx1)
% ts     : sampling time
% q_min  : lower joint limits (nx1)
% q_max  : upper joint limits (nx1)
%
% Output:
% q      : new joint configuration (nx1)

    % Euler integration
    q_new = q + q_dot*ts;
    q_new = max(q_min, min(q_new, q_max));

end
