classdef MotorCortex < handle
    %MOTORCORTEX Controls the actors of the agent
    %   Detailed explanation goes here
    
    properties(Access=private,Constant=true)
        
        % set maximum movement cell value
        MAX_MOVEMENT = 30;
        
    end
    
    methods
        
        function obj = MotorCortex()
            % The constructor
            
            %TODO?
            
        end
        
        function movement = update(~, movementCells)
            % This function receives the activity of movementCells [left, straight, right] and
            % calculates the resulting motor commands with values between 0 and 0.5. This vector can
            % later (in Agent class) be used for velocity calculations
            
            movement = [0 0];
            
            % add the straight values to the motor commands
            movement(1) = movementCells(2);
            movement(2) = movementCells(2);
            
            % add activity of turning cell to motor commands
            movement(1) = movement(1) + movementCells(1) ;
            movement(2) = movement(2) + movementCells(3) ;
            
            
            % normalize so that values are between 0 and 0.5 (for tan in Environment class)
            movement = movement ./ (2*max(movement));
            
        end
        
        
    end
    
end

