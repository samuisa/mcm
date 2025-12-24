%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>
    % (see notes)
    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    properties
        iTj_0
        jointType
        eTt
        jointNumber
        iTj
        q
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.eTt = eTt;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end
        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs:
            % q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)
            
            %TO DO
            self.q = q;

            for i=1:self.jointNumber
                
                if self.jointType(i) == 0
                    R = [cos(q(i)) -sin(q(i)) 0;
                         sin(q(i)) cos(q(i))  0;
                            0         0       1];
                    T = [R, [0;0;0]; 0, 0, 0, 1];
                
                    self.iTj(:,:,i) = self.iTj_0(:,:,i) * T;

                elseif self.jointType(i) == 1
                    R = eye(3);
                    r = [0 0 1]'*q(i);
                    T = [R, r; 0, 0, 0, 1];
                    self.iTj(:,:,i) = self.iTj_0(:,:,i) * T;
                else 
                error("invalid joint type")
                end 
            end
         end

        function [bTt] = getToolTransformWrtBase(self)
            %% getToolTransformWrtBase function
            % outputs
            % bTt : transformation matrix from the manipulator base to the
            % tool

            %TO DO

            bTe = self.getTransformWrtBase(self.jointNumber);

            bTt = bTe * self.eTt;
        end

        function [bTk] = getTransformWrtBase(self,k)
            %% getTransformWrtBase function
            % Inputs :
            % k: the idx for which computing the transformation matrix
            % outputs
            % bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

                eps = 1e-6;
    
                if k < 1 || k > self.jointNumber
                    error("Invalid index");
                end
            
                bTk = eye(4);  % identity (base frame)
            
                for i = 1:k
                    bTk = bTk * self.iTj(:,:,i);   % multiply in forward order
                end
    
                for i = 1:4
                    for j = 1:4
                        if abs(bTk(i,j)) < eps
                            bTk(i,j) = 0;
                        end
                    end
                end
            end
            %TO DO
        end

end


