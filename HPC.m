classdef HPC < handle
    %HPC models the hippocampus
    %   Detailed explanation goes here
    
    
    %
    % Parameters
    %
    properties (Access=private, Constant=true)
        
        % DEBUG MOD
        DEBUG_MODE = 1;
        
        % predefined radius of each place cell
        RADIUS_PC = 0.5;
        
        % duration of one time step
        DELTA_T = 0.25;
        
        
        %%% RING ATTRACTOR PARAMETERS %%%
        % set stripe cell tuning
        SIGMA_RING = 5;
        % direction
        DIRECTIONS = [10:10:180];
        % number of phases to calculate for each ring attractor
        %         PHASES = [0,1/5,2/5,3/5,4/5]
        PHASES = [0:1/360:1]
        % distance between two activity peaks
        SCALES = [20,35,50];
        % Velocity , basically depending on Environment
        VELOCITY = 6.25;
        
        
    end
    
    properties (Access=private, Constant=false)
        
        % the preferred direction of each cell starting at 0 to 359
        prefDirection360 = [];
        
        % Allocentric Goal direction (AGD) cells
        agdCells = [];
        
        % Head Direction (HD) cells
        hdCells = [];
        
        % Stripe cells
        stripeCells = []
        
        
        % stores the goal direction (in degrees)
        goal = 0;
        
        % stores the head direction
        hd;
        
        
        % number of cells in each layer
        n;
        
        % variance for each layer gaussian function
        sigma;
        
        % stores the current time
        currentTime;
        
        %%% Testing grid cell model %%%
        % value Dd of equation 2 (we have D_d value for each degree -> 360)
        direction_displacement = zeros(18,1)
        
        % store value of a singel stripe cell
        single_stripe = [];
        
        % for plotting
        figureStripeCells1 =[];
        figureStripeCells2 =[];
        figureStripeCells3 =[];
    end
    
    
    % Update the timestamp in each public method
    methods(Access=public)
        
        function obj = HPC(n,sigma)
            % Constructor for RSC. Set number of cells for each layer and
            % corresponding sigma
            
            % number of cells in each layer
            obj.n = n;
            % variance for each layer gaussian function
            obj.sigma = sigma;
            
            obj.currentTime = 0.0;
            
            obj.prefDirection360 = [0:(360/obj.n):360-1];
            
            
        end
        
        
        function agdCells = updateGoal(obj, goalPosition, agentPose)
            % This function receives the new goal postion and the agent
            % pose in x/y coordinates and updates the model accordingly
            % FOR NOW THE GOAL POSITION IS IN DEGREES
            
            % calculate goal direction
            obj.goal = obj.calculateGoalDirection(goalPosition,agentPose);
            
            % set new head direction
            obj.hd = agentPose(3);
            
            
            % update the complete model
            obj.updateModel();
            
            % receive tuning
            agdCells = obj.agdCells();
            
            % update time
            obj.currentTime = obj.currentTime + obj.DELTA_T;
            
        end
        
        
        
    end
    
    methods(Access=private)
        
        function updateModel(obj)
            % This function updates the complete cell model so that the
            % allocentric goal direction can be read out
            
            if isempty(obj.goal)
                disp('Goal not set');
                return;
            end
            
            % set tuning of agd cells
            for i=1:obj.n
                if ( exp(-(obj.prefDirection360(i)-obj.goal)^2/obj.sigma^2) ) > 0
                    obj.agdCells(i) = exp(-(obj.prefDirection360(i)-obj.goal)^2/obj.sigma^2);
                elseif( exp(-(360-obj.goal+obj.prefDirection360(i))^2/obj.sigma^2) >0 )
                    obj.agdCells(i) = exp(-(360+obj.prefDirection360(i)-obj.goal)^2/obj.sigma^2);
                else
                    obj.agdCells(i) = exp(-(360-obj.prefDirection360(i)+obj.goal)^2/obj.sigma^2);
                end
                
            end
            
            % set tuning of hd cells. Make sure that the x mod 360 requirement is fulfilled
            for i=1:obj.n
                
                if ( exp(-(obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2) ) > 0.001
                    obj.hdCells(i) = exp(-(obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2);
                elseif( exp(-(360-obj.hd+obj.prefDirection360(i))^2/obj.sigma^2) > 0.001 )
                    obj.hdCells(i) = exp(-(360+obj.prefDirection360(i)-obj.hd)^2/obj.sigma^2);
                else
                    obj.hdCells(i) = exp(-(360-obj.prefDirection360(i)+obj.hd)^2/obj.sigma^2);
                end
                
            end
            
            
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% TESTING grid cell model %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % calculate displacement for one specific direction
            % needs needs to be done for every direction?
            
            for direction_index=1:length(obj.DIRECTIONS)
                direction = obj.DIRECTIONS(direction_index);
                % equation 1 from paper
                v_d = cos(direction-obj.hd); %*obj.VELOCITY;
                % equation 2 from paper
                obj.direction_displacement(direction_index) = obj.direction_displacement(direction_index) + v_d;
                
                for scale_index=1:length(obj.SCALES)
                    scale = obj.SCALES(scale_index);
                    for phase_index= 1:length(obj.PHASES)
                        phase = obj.PHASES(phase_index);
                        % correct phase
                        phase = phase * scale;
                        
                        % equation 3 from paper
                        omega_dps = mod((obj.direction_displacement(direction_index) - phase) , scale);
                        % equation 4 from paper
                        obj.stripeCells(scale_index,direction_index,phase_index) = exp( - (min(omega_dps,scale-omega_dps)^2 / (2*(scale*0.07)^2)) );
                        
                    end
                    
                end
                
            end
            
            
            
            %
            
            % only show selected stripe cells 
%             obj.stripeCells = reshape(obj.stripeCells(1,3,:),1,[]);
            
            % Plot all Cell layers
            if obj.DEBUG_MODE
                if obj.currentTime > 0
                    obj.updatePlotCells();
                else
                    obj.initializePlotCells();
                end
            end
            
        end
        
        
        function goalDirection = calculateGoalDirection(~,goalPosition,agentPose)
            % This function calculates the allocentric goal direction
            
            goalDirection = atan2d(goalPosition(1) - agentPose(1), goalPosition(2) - agentPose(2));
            
        end
        
        function initializePlotCells(obj)
            
            
%             hold on;
%             
%             subplot(5,4,[11 12 15 16 19 20])
%             obj.figureStripeCells = plot([1:length(obj.stripeCells)],obj.stripeCells, 'r');
%             
%             obj.figureStripeCells.XDataSource = 'X_STRIPES';
%             obj.figureStripeCells.YDataSource = 'Y_STRIPES';
%             
%             sNeg = {sprintf('0%c', char(176))};
%             sNeg= [sNeg,sprintf('60%c', char(176))];
%             sNeg= [sNeg,sprintf('120%c', char(176))];
%             sNeg= [sNeg,sprintf('180%c', char(176))];
%             sNeg= [sNeg,sprintf('240%c', char(176))];
%             sNeg= [sNeg,sprintf('300%c', char(176))];
%             sNeg= [sNeg,sprintf('360%c', char(176))];
%             set(gca,'FontSize',13,'FontWeight','bold');
%             
%             set(gca,'XTickLabel',sNeg);
%             set(gca,'XTick',[0,60,120,180,240,300,360]);
%             set(gca,'YTickLabel',[]);
%             ylabel('Ring Attractor Activity');
%             xlabel('Phases (degree)');
%             ylim([0 1]);
%             title(sprintf('Stripe Cell Population Activity for scale %i and direction %i', obj.SCALES(1),obj.DIRECTIONS(3)));
%             
%             hold off;

            hold on;
            
            subplot(5,4,[11 12])
           [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
            z = obj.stripeCells(1,:,:);
            z = squeeze(permute(z,[2,1,3]));
            obj.figureStripeCells1  = pcolor(x,y,z);
            set(gca,'FontSize',10,'FontWeight','bold');
            set(gca,'XTick',[0,60,120,180,240,300,360]);
            set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
            set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
            ylabel('Tuned Diretions');
            title(sprintf('Stripe Cell Population Activity for scale %i ', obj.SCALES(1)));
            hold off;
            
            hold on;
            subplot(5,4,[15 16 ])
           [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
            z = obj.stripeCells(1,:,:);
            z = squeeze(permute(z,[2,1,3]));
            obj.figureStripeCells2  = pcolor(x,y,z);
            set(gca,'FontSize',10,'FontWeight','bold');
            set(gca,'XTick',[0,60,120,180,240,300,360]);
            set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
            set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
            ylabel('Tuned Diretions');
            title(sprintf('Stripe Cell Population Activity for scale %i', obj.SCALES(2)));
            hold off;
            
            hold on;
            subplot(5,4,[19 20 ])
           [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
            z = obj.stripeCells(1,:,:);
            z = squeeze(permute(z,[2,1,3]));
            obj.figureStripeCells3  = pcolor(x,y,z);
            set(gca,'FontSize',10,'FontWeight','bold');
            set(gca,'XTick',[0,60,120,180,240,300,360]);
            set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
            set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
            ylabel('Tuned Diretions');
            xlabel('Phases (degree)');
            title(sprintf('Stripe Cell Population Activity for scale %i i', obj.SCALES(3)));
            hold off;
            
            
            
            
        end
        
        function updatePlotCells(obj)
            % This function updates the plot
            z = obj.stripeCells(1,:,:);
            z = squeeze(permute(z,[2,1,3]));
            set(obj.figureStripeCells1, 'CData', z); 
            
            z = obj.stripeCells(2,:,:);
            z = squeeze(permute(z,[2,1,3]));
            set(obj.figureStripeCells2, 'CData', z); 
            
            z = obj.stripeCells(3,:,:);
            z = squeeze(permute(z,[2,1,3]));
            set(obj.figureStripeCells3, 'CData', z); 
            
            refreshdata(obj.figureStripeCells1,'caller');
            refreshdata(obj.figureStripeCells2,'caller');
            refreshdata(obj.figureStripeCells3,'caller');
        end
        
    end
    
end

