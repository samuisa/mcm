%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            % Jacobian of link i w.r.t. base
                        
            % Trasformazione del link i rispetto alla base
            eps = 1e-6;
            bTn = self.gm.getTransformWrtBase(i);
            
            for j = 1:self.gm.jointNumber
                
                bTj = self.gm.getTransformWrtBase(j);
                bJi(:, j) = zeros(6, 1);

                if self.gm.jointType(j) == 0   % giunto rotazionale
                    
                    % Parte angolare
                    bJi(1:3, j) = bTj(1:3, 3);
                    
                    % Parte lineare
                    bJi(4:6, j) = cross(bTj(1:3, 3), bTn(1:3, 4) - bTj(1:3, 4));
                
                elseif self.gm.jointType(j) == 1   % giunto prismatico
                    
                    % Parte lineare
                    bJi(4:6, j) = bTj(1:3, 3);
                end

                if(i<self.gm.jointNumber && j>i)
                    bJi(:, j) = [0;0;0;0;0;0];
                end
            
            end
            for i = 1:self.gm.jointNumber
                for j = 1:6
                    if abs(bJi(j,i)) < eps
                        bJi(j,i) = 0;
                    end
                end
            end
        end



        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO

            bJe = self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
            
            r = [0; 0; 0];
            re_7 = vecToSkew(r);

            J_A = bJe(1:3,:);
            J_L = bJe(4:6,:);

            bJe_L = J_L - (re_7*J_A);
            bJe_A = J_A;

            self.J(1:3, :) = bJe_A;
            self.J(4:6, :) = bJe_L;
                      
            self.J;
           
            
        end
    end
end

