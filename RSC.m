classdef RSC < handle
    %RSC models the retrosplenial cortex
    %   Detailed explanation goes here
    
    
    
    %    ___                         _                  %
    %   | _ \__ _ _ _ __ _ _ __  ___| |_ ___ _ _ ___    %
    %   |  _/ _` | '_/ _`| '  \/ -_)  _/ -_) '_(_-<     %
    %   |_| \__,_|_|\__,_|_|_|_\___|\__\___|_| /__/     %
    %                                                   %
    properties (Access=private, Constant=true)
        
        % DEBUG MOD
        DEBUG_MODE = 0;
        
        % duration of one time step
        DELTA_T = 0.25;
                
        % Signal to noise ratio
        CELL_DIRECTION_NOISE = 5;
    end
    
    properties (Access=private, Constant=false)
        
        % the preferred direction of each cell starting at 0 to 359
        prefDirection360 = [];
        % the preferred direction of each cell starting at -180 to 179
        prefDirection180 = [];
        
        % Allocentric Goal direction (AGD) cells
        agdCells = [];
        % Head Direction (HD) cells
        hdCells = [];
        % Egocentric Goal Direction (EGD) cells (also for decision making)
        egdCells = [];
        % Egocentric Goal Cue Direction (EGCD) cells received from PPC for
        % decision making
        egcdCells = [];
        % Route-centric information received from PPC for decision making
        pathComplete = [];
        pathProgress = [];
        
        % stores the head direction
        hd;
        
        
        % indicates the frame the agent must use:
        % 1: pure allocentric
        % 2: pure egocentric
        % 3: pure route-centric
        % 4: ego- & allocentric
        % 5: ego- & routecentric
        % 6: Ego- & route- & allocentric
        frameDecision = 4;
        
        % stores the current time
        currentTime;
        
        
        % number of cells in each layer (360 mod n == 0 && n mod 2 == 0 &&
        % n mod 3 == 0)
        n;
        
        % variance for each layer gaussian function
        sigma;
    end
    
    
    % Update the timestamp in each public method
    methods
        
        function obj = RSC(n,sigma)
            % Constructor for RSC. Set number of cells for each layer and
            % corresponding sigma
            
            % number of cells in each layer
            obj.n = n;
            % variance for each layer gaussian function
            obj.sigma = sigma;
            
            obj.currentTime = 0.0;
            
            obj.prefDirection360 = 0:(360/obj.n):360-1;
            obj.prefDirection180 = -180:(360/obj.n):180-1;
            
        end
        
        
        function [egdCells,frameDecision,hdCells] = updateCells(obj, agdCells, hd,pathComplete,pathProgress)
            % This function receives the new agdCells tuning and head direction and updates
            % the complete model accordingly. It returns the new EGD, the frame to use and the HD
            % cell tuning
            
            % set new agdCells
            obj.agdCells = agdCells;
            
            % set new head direction
            obj.hd = hd;
            
            % set path details
            obj.pathComplete = pathComplete;
            obj.pathProgress = pathProgress;
            
            % update the complete model
            obj.updateModel();
            
            % calculate new frame to use
            obj.updateFrameDecision();
            
            % receive frame decision
            frameDecision = obj.frameDecision;
            
            % receive tuning
            egdCells = obj.egdCells;
            
            % receive head direction
            hdCells = obj.hdCells;
            
            % update time
            obj.currentTime = obj.currentTime + obj.DELTA_T;
            
        end
        
        
    end
    
    methods(Access=private)
        
        function updateModel(obj)
            % This function updates the complete cell model so that the new
            % egocentric goal direction can be read out
            % shift the tuning of agd with hd to get egd
            
             % set tuning of hd cells. Make sure that the x mod 360 requirement is fulfilled
             
%              obj.sigma = 80;
            for i=1:obj.n
                
                if ( exp(-(obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2) ) > 0.001
                    obj.hdCells(i) = exp(-(obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2);
                elseif( exp(-(360-obj.hd+obj.prefDirection360(i))^2/obj.sigma^2) > 0.001 )
                    obj.hdCells(i) = exp(-(360+obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2);
                else
                    obj.hdCells(i) = exp(-(360-obj.prefDirection360(i)+obj.hd)^2/obj.sigma^2);
                end
                
            end
%             obj.sigma = 50;
            
%             obj.hdCells = awgn(obj.hdCells,20);
            obj.hdCells(obj.hdCells < 0 ) = 0.0;
            
            obj.hdCells = obj.hdCells ./ max(obj.hdCells(:));
            
            % Measure the distance between active HD and AGD cells and only
            % activate a EGD cell if distance is corresponding
            [~,k] = max(obj.agdCells);
            [~,j] = max(obj.hdCells);
            for i=1:obj.n
                obj.egdCells(i) = exp(-(obj.prefDirection180(i))^2/obj.sigma^2);
            end           
            obj.egdCells = circshift(obj.egdCells,[0,k-j]);
             % add noise 
            obj.egdCells = circshift(obj.egdCells,[0,round(randn()*obj.CELL_DIRECTION_NOISE)]);
            
%             obj.egdCells = awgn(obj.egdCells,20);
            obj.egdCells(obj.egdCells < 0 ) = 0.0;
            obj.egdCells = obj.egdCells ./ max(obj.egdCells(:));

            
            % Plot variables
            if (obj.DEBUG_MODE)
                
                hFig = figure();
                set(hFig, 'Position', [400 400 800 800])
                
                subplot(3,1,1)
                plot(obj.prefDirection360,obj.agdCells);
                ylim([-0.5 1.5]);
                xlim([0 359]);
                title('Allocentric Goal Direction');
                
                subplot(3,1,2)
                plot(obj.prefDirection360,obj.hdCells);
                ylim([-0.5 1.5]);
                xlim([0 359]);
                title('Head Direction');
                
                subplot(3,1,3)
                plot(obj.prefDirection180,obj.egdCells);
                ylim([-0.5 1.5]);
                xlim([-180 179]);
                title('Egocentric Goal Direction');
                
                
            end
            
        end
        
        
        function updateFrameDecision(obj)
            % This function calculates which frame should be used for further
            % actions
            
            
            if (obj.currentTime > 1.0)
                obj.frameDecision = 4;
            end

            
        end
        
        
        
        
    end
    
end

