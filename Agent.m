classdef Agent<handle
    %AGENT Represents the agent
    %   This class simulates the agent. The agent acts in the environment, preceives signals and
    %   uses other classes i.e. PPC to do calculations
    
    
    
    properties(Access=private, Constant=true)
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % LET THEM BE CONSTANTS JUST FOR NOW !
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % duration of one time step
        DELTA_T = 0.25;
        
        % number of cells in each layer (360 mod N == 0 && N mod 2 == 0 && N mod 3 == 0)
        N = 360;
        
        % variance for each layer gaussian function
        SIGMA = 50;
        
        % if true assume we run only a simulation
        SIMULATION = 1;
        
        SENSOR_NOISE = 5;
        
    end
    
    properties(Access=public)
        
    end
    
    
    properties(Access=private, Constant=false)
        
        %%%%%%% TESTING PURPOSE %%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % stores the goal direction
        goal = [];
        
        % stores the cues
        cues = [];
        
        % allocentric position of agent [x,y]
        position = [0,0];
        
        % allocentric HD of the agent in degrees
        hdAgent = 0;
        
        % the velocity of the agent
        velocity = 5;
        
        % the path that the agent has travelled so far (used for path integration). TODO Maybe just
        % store movements?
        pathTravelled = [];
        
        % the starting position of the agent
        startPosition = [0,0];
        
        % stores the currently active place cell
        currentPC = [];
        
        % stores the goal place cell
        goalPC = [];
        
        % stores the current time
        currentTime;
        
        % the HPC object
        hpc = [];
        
        % the RSC object
        rsc = [];
        
        % the PPC object
        ppc = [];
        
        % Allocentric Goal direction (AGD) cells
        agdCells = [];
        % Head Direction (HD) cells
        hdCells = [];
        % Egocentric Goal Direction (EGD) cells (also for decision making)
        egdCells = [];
        % Egocentric Goal Cue Direction (EGCD) cells received from PPC for decision making
        egcdCells = [];
        % Route-centric information received from PPC for decision making
        pathComplete = [];
        pathProgress = [];
        
        % the movement to execute
        movement;
        
        % the frame decision
        frameDecision
        
        
        
    end
    
    
    methods(Access=public)
        
        function obj = Agent(startPosition,goal,cues)
            % Constructor for the class Agent. It receives the startPosition as a parameter
            
            obj.currentTime = 0.0;
            
            % set start position
            obj.startPosition = startPosition;
            obj.position = startPosition;
            obj.pathTravelled = startPosition;
            
            obj.hdAgent = obj.updateHD();
            
            obj.cues = cues;
            obj.goal = goal;
            
            % create the brain areas on start up
            obj.hpc = HPC(obj.N,obj.SIGMA);
            obj.rsc = RSC(obj.N,obj.SIGMA);
            obj.ppc = PPC(obj.N,obj.SIGMA);
            
        end
        
        function initializeAgentCells(obj)
            % This function helps the agent to initialize all cells and prepare it for first
            % movements
            
            % Get head direction from measurements .... obj.hdAgent = obj.hd;
            obj.hdAgent = 0;
            
            agentPose = [obj.startPosition, obj.hdAgent];
            
            % calculate/sense the direction and distance of each cue
            cuesDirectionDistance = obj.senseCues(obj.cues,obj.position,[0 0]);
            
            % HPC sends allocentric goal direction tuning to RSC
            obj.agdCells = obj.hpc.updateGoal(obj.goal,agentPose);
            % RSC sends egocentric goal direction and default frame to use to PPC
            [obj.egdCells,obj.frameDecision,obj.hdCells] = obj.rsc.updateCells(obj.agdCells,obj.hdAgent,obj.pathComplete,obj.pathProgress);
            % PPC calculates movements and returns the EGCD cells (egocentric navigation) and the
            % path for further processing in the RSC
            [obj.movement] = obj.ppc.updateCells(obj.egdCells,cuesDirectionDistance,obj.hdCells);
            
        end
        
        function [movement,orientation] = act(obj,agentPosition,agentOrientation,goal)
            % This function receives the new position of the agent in the environment and calculates
            % the next movement using the brain model. It then returns the movement.
            
            obj.position = agentPosition;
            obj.pathTravelled = [obj.pathTravelled ; agentPosition];
            
            % from last
            
            
            % calculate/sense the direction and distance of each cue
            cuesDirectionDistance = obj.senseCues(obj.cues,obj.position,goal);
            
            
            % Get head direction from measurements (updateHD) or for simulation just use the
            % orientation given by the environment ....
            if obj.SIMULATION
                obj.hdAgent = agentOrientation;
            else
                obj.hdAgent = obj.updateHD();
            end
            
            agentPose = [obj.position, obj.hdAgent];
            obj.goal = goal;
            
            % HPC sends allocentric goal direction tuning to RSC
            obj.agdCells = obj.hpc.updateGoal(obj.goal,agentPose);
            % RSC sends egocentric goal direction and frame to use to PPC
            [obj.egdCells,obj.frameDecision,obj.hdCells] = obj.rsc.updateCells(obj.agdCells,obj.hdAgent,obj.pathComplete,obj.pathProgress);
            % PPC calculates EGOCENTRIC movement coommands
            [obj.movement] = obj.ppc.updateCells(obj.egdCells,cuesDirectionDistance,obj.hdCells);
            
            % Use velocity to get actual movement commands
            movement = obj.movement;
            
            
            % set movement of agent manually
%             if obj.currentTime < 1.0
%                 
%                 movement = [0.4 0.5];
%                 
%             elseif obj.currentTime < 30
%                 movement = [0.5 0.5];
%             elseif obj.currentTime < 34
%                 movement = [0.1 0.5];
%             else
%                 movement = [0.5 0.5]
% 
%             end
            
            
            % return also current orientation
            orientation = obj.hdAgent;
            
            % update the current time
            obj.currentTime = obj.currentTime + obj.DELTA_T;
            
            
           
            
            
        end
        
    end
    
    
    methods(Access=private)
        
        function cues = senseCues(obj, cues, agentPosition,goal)
            % This function uses calculates the egocentric orientation and distance of each cue
            % according to the agent's position and orientation
            
            distances = pdist2(agentPosition,goal);
            allocentricOrientations = atan2d(goal(:,1) - agentPosition(1), goal(:,2) - agentPosition(2));
            cues = [ allocentricOrientations, distances' ];
            cues(:,1) = cues(:,1) - obj.updateHD();
            %             cues = cues + cues .* randn() .* obj.SENSOR_NOISE;
            
            for i=1:length(cues(:,1))
                if (cues(i,1) < -360)
                    cues(i,1) = mod(cues(i,1),-180);
                elseif (cues(i,1) < -180)
                    cues(i,1) = mod(cues(i,1),180);
                end
                
                if (cues(i,1) >= 360)
                    cues(i,1) = mod(cues(i,1),180);
                elseif (cues(i,1) >= 180)
                    cues(i,1) = mod(cues(i,1),-180);
                end
                
            end
            
        end
        
        
        function agentHD = updateHD(obj)
            % This functions determines the HD of the agent. It should do that in reading the compass
            % sensor measurement. Not used in simulation only ....
            
            if (obj.currentTime == 0.0 )
                agentHD = 0;
            else
                % calculate new heading
                agentHD = atan2d(obj.pathTravelled(end,1) - obj.pathTravelled(end-1,1),obj.pathTravelled(end,2) - obj.pathTravelled(end-1,2));
                % no negativ values as HD
                agentHD = mod(agentHD,360);
            end
            
            
        end
        
        
    end
    
end

