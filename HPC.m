classdef HPC < handle
    %HPC models the hippocampus
    %   Detailed explanation goes here
    
    
    %
    % Parameters
    %
    properties (Access=private, Constant=true)
        
        % duration of one time step
        DELTA_T = 0.25;
        
        
        
        % DISPLAY MODE
        % change that value to show only the environment (0), the
        % environment together with the rate map of defined stripe cell (1)
        % or the environment together with the rate map and all ring
        % attractors (2). IMPORTANT: If displaying all ring attractors one
        % should increase the number of phases to display e.g. [0:1/100:1]
        DISPLAY_MODE = 1;
        
        % choose the stripe cell for that you want to record the activity
        % in the environment (direction, scale, phase)
        STRIPE_CELL = {9,3,1}
        
        %%% STRIPE CELL TUNING PARAMETERS %%%
        % direction
        DIRECTIONS = [10:10:180];
        % number of phases to calculate for each ring attractor
        PHASES = [0,1/5,2/5,3/5,4/5]
        %PHASES = [0:1/100:1]
        % distance between two activity peaks
        SCALES = [20,35,50];
        
        % number of grid cells
        GC_NUMBER = 10;
        
    end
    
    properties (Access=private, Constant=false)
        
        % the preferred direction of each cell starting at 0 to 359
        prefDirection360 = [];
        
        % Allocentric Goal direction (AGD) cells
        agdCells = [];
        
        % Head Direction (HD) cells
        hdCells = [];
        
        % Stripe cells
        stripeCells = [];
        
        % grid cells
        grid_cell = [];
        
        % output signal
        G_output = [];
        
        % weights stripe cells to grid cells
        w =[];
        
        % stores the goal direction (in degrees)
        goal = 0;
        
        % stores the head direction
        hd;
        
        agentPosition = [0 0];
        
        % Velocity of the agent (default 10)
        agentVelocity = 10;
        
        % number of cells in each layer
        n;
        
        % variance for each layer gaussian function
        sigma;
        
        % stores the current time
        currentTime;
        
        %%% Testing grid cell model %%%
        singleStripeCell =zeros(500)
        % value Dd of equation 2 (we have D_d value for each degree -> 360)
        direction_displacement = zeros(18,1)
        
        % store value of a singel stripe cell
        single_stripe = [];
        
        % for plotting
        figureStripeCells1 =[];
        figureStripeCells2 =[];
        figureStripeCells3 =[];
        figureStripeCellMap =[];
    end
    
    
    % Update the timestamp in each public method
    methods(Access=public)
        
        function obj = HPC(n,sigma,agentVelocity)
            % Constructor for RSC. Set number of cells for each layer and
            % corresponding sigma
            
            % number of cells in each layer
            obj.n = n;
            % variance for each layer gaussian function
            obj.sigma = sigma;
            
            obj.agentVelocity = agentVelocity;
            
            obj.currentTime = 0.0;
            
            obj.prefDirection360 = [0:(360/obj.n):360-1];
            
            % initialize weigths stripe cells to grid cells
            obj.w = rand(length(obj.DIRECTIONS),length(obj.SCALES),length(obj.PHASES),obj.GC_NUMBER)./10;
            
            % initialize grid cells with zeros??
            obj.grid_cell = zeros(length(obj.SCALES),obj.GC_NUMBER);
            
            obj.G_output = zeros(obj.GC_NUMBER,length(obj.SCALES));
            
            
        end
        
        
        function agdCells = updateGoal(obj, goalPosition, agentPose)
            % This function receives the new goal postion and the agent
            % pose in x/y coordinates and updates the model accordingly
            % FOR NOW THE GOAL POSITION IS IN DEGREES
            
            % calculate goal direction
            obj.goal = obj.calculateGoalDirection(goalPosition,agentPose);
            
            % set new head direction
            obj.hd = agentPose(3);
            
            % set new agent position
            obj.agentPosition = agentPose(1:2);
            
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
            %%% Stripe cell model       %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % calculate displacement
            v_ds = cosd(obj.DIRECTIONS -obj.hd) *obj.agentVelocity * obj.DELTA_T;
            obj.direction_displacement = obj.direction_displacement + v_ds';
            
            % calculate omega
            phases = obj.PHASES .* obj.SCALES';
            tmp = bsxfun(@minus,obj.direction_displacement,permute(phases,[3 1 2]));
            omega_dps = mod(tmp , obj.SCALES);
            % calculate stripe cells activity
            obj.stripeCells = exp( - (min(omega_dps,obj.SCALES-omega_dps).^2 ./ (2.*(obj.SCALES.*0.07).^2)) );
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% Grid cell model         %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%% PARAMETERS %%%
            % passive decay value
            A = 10;
            % feedforward excitatory input
            alpha = 100;
            % lateral inhibition
            beta = 30;
            % output threshold
            gamma = 0.25;
            % learning rate
            lambda_w = 0.01;
            
            
            
            % sum over stripe cells x weights
            sum_stripe_weights = squeeze(sum(sum(repmat(obj.stripeCells,1,1,1,obj.GC_NUMBER) .* obj.w,1),3))';
            % sum over output signal G , but remove jth column
            sum_g = squeeze(sum(repmat(obj.G_output,1,1,obj.GC_NUMBER),1))' - obj.G_output;
            % calculate delta_g (equation 5)
            delta_g = -A .* obj.grid_cell' + (1-obj.grid_cell') .* ( alpha .* sum_stripe_weights ) - obj.grid_cell' .* ( beta .* sum_g) ;
            % integrate over delta_g x DELTA_T
            obj.grid_cell = obj.grid_cell + delta_g' .* obj.DELTA_T;
            
            % calculate output signal G (equation 6)
            obj.G_output = max(0,obj.grid_cell'-gamma) ./ (1-gamma);
            
            % sum over stripe cells directions
            sum_stripe_cells =permute(squeeze(sum(repmat(obj.stripeCells,1,1,1,length(obj.DIRECTIONS)),1)),[3 1 2]) - obj.stripeCells;
            % sum over stripe cells phases
            sum_stripe_cells =permute(squeeze(sum(repmat(sum_stripe_cells,1,1,1,length(obj.PHASES)),3)),[1 2 3]) - sum_stripe_cells;
            % calculate delta_w (equation 9)
            delta_w = lambda_w .* permute(repmat(obj.G_output,1,1,length(obj.PHASES),length(obj.DIRECTIONS)),[4 2 3 1]) .* ( repmat(obj.stripeCells,1,1,1,obj.GC_NUMBER) - obj.w .* sum_stripe_cells);
            % integrate over delta_w x DELTA_T
            obj.w = obj.w + delta_w .* obj.DELTA_T;
            
            %             for scale=1:length(obj.SCALES)
            %                 for gc_ind=1:obj.GC_NUMBER
            %
            %                     % change of grid cell activity j of scale s (equation 5)
            %
            %                     % HELPER sum up stripe cell activity x weights
            %                     sum_stripe_cells = sum(sum(squeeze(obj.stripeCells(:,scale,:) .* obj.w(:,scale,:,gc_ind))));
            %                     g_js = obj.grid_cell(scale,gc_ind);
            %                     delta_g_js = -A * g_js + (1-g_js) * (alpha * sum_stripe_cells) - g_js * (beta * sum(obj.G_output(1:end ~= gc_ind,scale)));
            %                     obj.grid_cell(scale,gc_ind) = obj.grid_cell(scale,gc_ind) + delta_g_js * obj.DELTA_T;
            %                     obj.G_output(gc_ind,scale) = max(0,obj.grid_cell(scale,gc_ind)-gamma) ./ (1-gamma);
            %                 end
            %             end
            %
            % %             obj.G_output = max(0,obj.grid_cell'-gamma) ./ (1-gamma);
            %
            %             for scale=1:length(obj.SCALES)
            %                 for gc_ind=1:obj.GC_NUMBER
            %
            %                     % weight update (equation 9)
            %                     for i=1:length(obj.DIRECTIONS)
            %                         for ii=1:length(obj.PHASES)
            %
            %                             sum_stripe_cells = sum(sum(obj.stripeCells(1:end ~= i,scale,1:end ~= ii)));
            %
            %                             delta_w = lambda_w * obj.G_output(gc_ind,scale) * ( obj.stripeCells(i,scale,ii) - obj.w(i,scale,ii,gc_ind) * sum_stripe_cells);
            %
            %                             obj.w(i,scale,ii,gc_ind) = obj.w(i,scale,ii,gc_ind) + delta_w * obj.DELTA_T;
            %                         end
            %                     end
            %
            %                 end
            %             end
            
            
            
            %           obj.grid_cell = obj.grid_cell ./ max(obj.grid_cell) ;
            
            % calculate output signale
            %
            
            %                         % store response of singel stripe cell
            %                         x = uint16(obj.agentPosition(1));
            %                         y = uint16(obj.agentPosition(2));
            %                         % make sure agent is within the environment
            %                         if x > 0 && y > 0 && x < 500 && y < 500
            %                             % create index
            %                             indexMA = sub2ind(size(obj.stripeCells),obj.STRIPE_CELL{:});
            %                             obj.singleStripeCell(x,y) = obj.singleStripeCell(x,y) + obj.stripeCells(indexMA);
            %                         end
            
            
            
            % store response of singel grid cell
            x = uint16(obj.agentPosition(1));
            y = uint16(obj.agentPosition(2));
            % make sure agent is within the environment
            if x > 0 && y > 0 && x < 500 && y < 500
                % create index
                indexMA = sub2ind(size(obj.grid_cell),3,2);
                obj.singleStripeCell(x,y) = obj.singleStripeCell(x,y) + obj.G_output(indexMA);
            end
            
            
            
            % Plot all Cell layers
            if obj.DISPLAY_MODE
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
            
            
            
            if obj.DISPLAY_MODE > 0
                
                hold on;
                axis = subplot(5,4,[ 15 16 19 20]);
                
                % shift plot downwards
                pos = get( axis, 'Position' );
                pos(2) = pos(2)-0.05;
                set(axis, 'Position', pos ) ;
                obj.figureStripeCellMap= pcolor(obj.singleStripeCell');
                set(obj.figureStripeCellMap, 'EdgeColor', 'none');
                title(sprintf('Rate map for stripe cell:  %i0 %i %i ', cell2mat(obj.STRIPE_CELL)));
                hold off;
                
            end
            
            if obj.DISPLAY_MODE > 1
                
                hold on;
                
                subplot(5,4,[3 4])
                [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
                z = obj.stripeCells(:,1,:);
                z = squeeze(permute(z,[2,1,3]));
                obj.figureStripeCells1  = pcolor(x,y,z);
                set(gca,'FontSize',10,'FontWeight','bold');
                set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
                set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
                ylabel('Tuned Diretions');
                title(sprintf('Stripe Cell Population Activity for scale %i ', obj.SCALES(1)));
                hold off;
                
                hold on;
                subplot(5,4,[7 8 ])
                [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
                z = obj.stripeCells(:,2,:);
                z = squeeze(permute(z,[2,1,3]));
                obj.figureStripeCells2  = pcolor(x,y,z);
                set(gca,'FontSize',10,'FontWeight','bold');
                set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
                set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
                ylabel('Tuned Diretions');
                title(sprintf('Stripe Cell Population Activity for scale %i', obj.SCALES(2)));
                hold off;
                
                hold on;
                subplot(5,4,[11 12 ])
                [x,y] = meshgrid(1:length(obj.PHASES),1:length(obj.DIRECTIONS));
                z = obj.stripeCells(:,3,:);
                z = squeeze(permute(z,[2,1,3]));
                obj.figureStripeCells3  = pcolor(x,y,z);
                set(gca,'FontSize',10,'FontWeight','bold');
                set(gca,'YTick',obj.DIRECTIONS(1:2:end)/10);
                set(gca,'YTickLabel',obj.DIRECTIONS(1:2:end));
                ylabel('Tuned Diretions');
                xlabel('Phases (degree)');
                title(sprintf('Stripe Cell Population Activity for scale %i', obj.SCALES(3)));
                hold off;
                
            end
            
            
            
            
        end
        
        function updatePlotCells(obj)
            % This function updates the plot
            
            if obj.DISPLAY_MODE > 0
                % update plot every 100 time steps
                if mod(obj.currentTime,20) ==0
                    
                    
                    set(obj.figureStripeCellMap,'CData',obj.singleStripeCell');
                end
            end
            
            if obj.DISPLAY_MODE > 1
                
                z = obj.stripeCells(:,1,:);
                z = squeeze(permute(z,[2,1,3]));
                set(obj.figureStripeCells1, 'CData', z);
                
                z = obj.stripeCells(:,2,:);
                z = squeeze(permute(z,[2,1,3]));
                set(obj.figureStripeCells2, 'CData', z);
                
                z = obj.stripeCells(:,3,:);
                z = squeeze(permute(z,[2,1,3]));
                set(obj.figureStripeCells3, 'CData', z);
                
                
            end
            
            refreshdata(obj.figureStripeCellMap,'caller');
            
        end
        
    end
    
end

