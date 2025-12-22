%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            bTt = self.gm.getToolTransformWrtBase();
            tTg = inv(bTt) * bTg;
            [h, theta] = RotToAngleAxis(tTg(1:3, 1:3));
            
            % error components wrt t
            rho_tg = h*theta;
            r_tg = tTg(1:3,4);

            % error components wrt base
            b_rho_tg = bTt(1:3,1:3) * rho_tg;
            b_r_tg = bTt(1:3,1:3) * r_tg;

            % overall error wrt base
            b_e_tg = [b_rho_tg ; b_r_tg];
            
            % Lambda gain matrix
            lambda = [    self.k_a*eye(3),      zeros(3,3)
                            zeros(3,3),      self.k_l*eye(3)     ];

            % x_dot computation (if the goal frame isn't moving)
            x_dot = lambda * b_e_tg;
        end
    end
end

