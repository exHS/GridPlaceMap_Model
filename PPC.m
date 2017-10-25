classdef PPC < handle
    %PPC models the posterior parietal cortex
    %   Detailed explanation goes here
    
    
    
    %
    % PARAMETERS
    %
    properties (Access=private,Constant=true)
        
        % activation level for the excitatory connection from egd to ecd
        ACTIVATION_LEVEL = 0.3;
        
        % sigma value of the excitatory & inhibitory connection matrix
        SIGMA_EXCIT_INHIBIT_MATRIX = 80;
        
        % duration of one time step
        DELTA_T = 0.25;
        
        % DEBUG MOD
        DEBUG_MODE = 0;
        
        % movementCell connection area in percent (defines how many EGD cells should
        % be connected to left resp. right movement cells)
        % This number is HIGHLY relevant for chosing the next movement
        EGD_MOVEMENT_CONNECTION = 0.35;
        
        % Signal to noise ratio
        CELL_DIRECTION_NOISE = 5;
        
        SAMPLE_RATE_HD = 2;
        
        SAMPLE_RATE_EGD = 2;
        
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
        % Egocentric Goal Direction (EGD) Cells
        egdCells = [];
        
        ecdCells = [];
        
        % Movement Cells [left,straight;right]
        movementCells = [];
        
        % store the actual movement (direction vector?) returned by the motor cortex
        movement = [];
        
        % the motor cortex object
        motorCortex = [];
        
        % Excitatory connection weights from EGD to ECD
        EGDtoECDexcitatory_weights = [];
        % Inhibitory connection weights from EGD to ECD
        EGDtoECDinhibitory_weights = [];
        % Connection from EGCD to Movement cells
        EGCDtoMovementCells_weights = [];
        % Connection from EGD to Movement cells
        EGDtoMovementCells_weights = [];
        
        % stores the perceptible cues with direction and distance [
        % [10,1.5]; [-40,1.2]; [130,1.8] ];
        cues = [];
        
        
        % number of cells in each layer (360 mod n == 0 && n mod 2 == 0 &&
        % n mod 3 == 0)
        n;
        
        % variance for each layer gaussian function
        sigma;
        
        
        
        % stores the current time
        currentTime;
        
        
        %%% PLOTTING STUFF %%%
        figureHD = [];
        figureEGD = [];
        figureConjunctiveCells = [];
        figureMovement = [];
        
        
    end
    
    
    % Update the timestamp in each public method
    methods
        
        function obj = PPC(n,sigma)
            % Constructor for the PPC. Set number
            % of cells for each layer and the variance
            
            obj.n = n;
            obj.sigma = 40;
            
            obj.prefDirection360 = 0:(360/obj.n):360-1;
            obj.prefDirection180 = -180:(360/obj.n):180-1;
            
            obj.currentTime = 0.0;
            
            obj.agdCells = zeros(obj.n,1);
            obj.hdCells = zeros(obj.n,1);
            obj.movementCells = zeros(3,1);
            
            obj.EGDtoECDexcitatory_weights = zeros(obj.n,obj.n);
            obj.EGDtoECDinhibitory_weights = zeros(obj.n,obj.n);
            obj.EGCDtoMovementCells_weights = zeros(obj.n,1);
            
            % create motor cortex on start up
            obj.motorCortex = MotorCortex();
            
            % If possible load the connection weight matrices
            if ( exist('EGDtoECDexcitatoryWeights.mat','file') && exist('EGDtoECDinhibitoryWeights.mat','file'))
                disp('Loading connection weights....')
                obj.EGDtoECDexcitatory_weights = struct2array(load('EGDtoECDexcitatoryWeights.mat','EGDtoECDexcitatory_weights'));
                obj.EGDtoECDinhibitory_weights = struct2array(load('EGDtoECDinhibitoryWeights.mat','EGDtoECDinhibitory_weights'));
            else
                % generate excitatory connection from each egd cell to all ecd
                % cells. Make sure that the x mod 360 requirement is fulfilled
                for i=1:obj.n
                    for j=1:obj.n
                        if( exp(-(360-obj.prefDirection360(i)+obj.prefDirection360(j))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2) > exp(-(360-obj.prefDirection360(j)+obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2) &&...
                                exp(-(360-obj.prefDirection360(i)+obj.prefDirection360(j))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2) > exp(-(obj.prefDirection360(j)-obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2) )
                            obj.EGDtoECDexcitatory_weights(i,j) = exp(-(360+obj.prefDirection360(j)-obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2);
                        elseif ( exp(-(obj.prefDirection360(j)-obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2)  > exp(-(360-obj.prefDirection360(j)+obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2) )
                            obj.EGDtoECDexcitatory_weights(i,j) = exp(-(obj.prefDirection360(j)-obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2);
                        else
                            obj.EGDtoECDexcitatory_weights(i,j) = exp(-(360-obj.prefDirection360(j)+obj.prefDirection360(i))^2/obj.SIGMA_EXCIT_INHIBIT_MATRIX^2);
                        end
                    end
                end
                
                % generate inhibitory connections from each egd cell to all ecd
                % cells.
                obj.EGDtoECDinhibitory_weights = -obj.EGDtoECDexcitatory_weights +1;
                
                EGDtoECDexcitatory_weights = obj.EGDtoECDexcitatory_weights;
                EGDtoECDinhibitory_weights = obj.EGDtoECDinhibitory_weights;
                
                save('EGDtoECDexcitatoryWeights.mat','EGDtoECDexcitatory_weights');
                save('EGDtoECDinhibitoryWeights.mat','EGDtoECDinhibitory_weights');
                
            end
            
            
            
            % generate connection from egd to movement cells (cos tuned).
            % This may need adaptation: which ecdcell is connected to which
            % movementCell ? Is it reasonable that left & right turn cells
            % are active simulatneously ? If not, all expect of one peak in
            % the ecd cells should be suppressed -> increase activation
            % level.
            
            % cosinus tuned connection weights
            % obj.EGCDtoMovementCells_weights = (cos((2*pi .*
            % obj.prefDirection360) ./ max(obj.prefDirection360)) +1) ./2;
            
            % linear connection weights
            obj.EGCDtoMovementCells_weights = ones(obj.n,1);
            % for now it's the as the connection from EGDC to movement cells
            obj.EGDtoMovementCells_weights = ones(obj.n,1);
            
        end
        
        
        
        function [movement] = updateCells(obj, egdCells, cues, hdCells)
            % Use the given EGD activity to update all cell layers and
            % produce new motor commands according to the frame decision
            
            
            obj.hdCells = hdCells;
            
            % Set new tuning
            obj.cues = cues;
            obj.egdCells = egdCells;
            
            
            % update the model
            obj.updateModel();
            
            
            % receive new tuning
            movement = obj.movement;
            
            
            
            % update time
            obj.currentTime = obj.currentTime + obj.DELTA_T;
            
        end
        
        
        
    end
    
    methods(Access=private)
        
        function updateModel(obj)
            % This function updates the complete PCC
            
            % check if there are perceptible cues
            if isempty(obj.cues)
                disp(' No cues perceptible');
            end
            
            if all(obj.egdCells == 0)
                disp('No EGD cell activity');
            end
            
            
            % Set ecdCells activity
            obj.ecdCells = zeros(obj.n,1);
            for i=1:obj.n
                for j=1:size(obj.cues,1)
                    
                    % Use only cues that are not too far away
                    if ( exp(-(obj.prefDirection180(i)-obj.cues(j,1))^2/obj.sigma^2) ) > 0
                        obj.ecdCells(i) = obj.ecdCells(i) +   exp(-(obj.prefDirection180(i)-obj.cues(j,1))^2/obj.sigma^2);
                    elseif( exp(-(180-obj.cues(j,1)+obj.prefDirection180(i))^2/obj.sigma^2) >0 )
                        obj.ecdCells(i) = obj.ecdCells(i) +   exp(-(-360-obj.prefDirection180(i)+obj.cues(j,1))^2/obj.sigma^2);
                    else
                        obj.ecdCells(i) = obj.ecdCells(i) +  exp(-(obj.prefDirection180(i)-obj.cues(j,1)-360)^2/obj.sigma^2);
                    end
                end
            end
            
            
            % call function to use ego- & allocentric navigation
            obj.movementCells = obj.useEgoAllocentricFrame();
            
            
            % call the motor cortex in order to update the actual movement
            obj.movement = obj.callMotorCortex(obj.movementCells);
            
            
            % Plot all Cell layers
            if obj.DEBUG_MODE
                if obj.currentTime > 0
                    obj.updatePlotCells();
                else
                    obj.initializePlotCells();
                end
            end
        end
        
        
        function movementCells = useEgoAllocentricFrame(obj)
            % Uses a combination of egocentric and allocentric frame to
            % calculate next movement.
            % lower  45% of ECD cells is connected to left turns
            % middle 1/3 part of ECD cells is connected to forward movement
            % upper 45% of ECD cells is connected to right turns
            movementCells = zeros(3,1);
            for i=1:obj.n
                if ( i <= obj.n*0.5)
                    movementCells(1) = movementCells(1) + obj.EGCDtoMovementCells_weights(i) * obj.ecdCells(i);
                end
                
                if ( i>=obj.n*obj.EGD_MOVEMENT_CONNECTION  && i<= obj.n*(1 - obj.EGD_MOVEMENT_CONNECTION))
                    movementCells(2) = movementCells(2) + obj.ecdCells(i);
                end
                
                if ( i>=obj.n*(1 - 0.5))
                    movementCells(3) = movementCells(3) + obj.EGCDtoMovementCells_weights(i) * obj.ecdCells(i);
                end
                
            end
            
            
            
        end
        
        function movement = callMotorCortex(obj,movementCells)
            % This function uses the stored information in the movement
            % cell to call the MotorCortex and receives a EGOCENTRIC direction vector
            
            movement = obj.motorCortex.update(movementCells);
            
        end
        
        
        
        function initializePlotCells(obj)
            % This function is for debugging
            subplot(5,4,[3 4 7 8])
            
            
            %             s = {sprintf('0%c', char(176))};
            %             s = [s,sprintf('45%c', char(176))];
            %             s = [s,sprintf('90%c', char(176))];
            %             s = [s,sprintf('135%c', char(176))];
            %             s = [s,sprintf('180%c', char(176))];
            %             s = [s,sprintf('225%c', char(176))];
            %             s = [s,sprintf('270%c', char(176))];
            %             s = [s,sprintf('315%c', char(176))];
            %             s = [s,sprintf('360%c', char(176))];
            %
            %             obj.figureEGD = bar(downsample(obj.prefDirection180,obj.SAMPLE_RATE_EGD), downsample(obj.egdCells,obj.SAMPLE_RATE_EGD));
            %             ylim([-0.0 1.5]);
            %             xlim([-180 180]);
            %             %             title('Egocentric Cue Direction');
            %             obj.figureEGD.XDataSource = 'X_EGD';
            %             obj.figureEGD.YDataSource = 'Y_EGD';
            %             sNeg = {sprintf('-150%c', char(176))};
            %             sNeg= [sNeg,sprintf('-75%c', char(176))];
            %             sNeg= [sNeg,sprintf('-0%c', char(176))];
            %             sNeg= [sNeg,sprintf('75%c', char(176))];
            %             sNeg= [sNeg,sprintf('150%c', char(176))];
            %             set(gca,'FontSize',13,'FontWeight','bold');
            %
            %             set(gca,'XTickLabel',sNeg);
            %             set(gca,'XTick',[-150,-75,-0,75,150]);
            %             set(gca,'YTickLabel',[]);
            %             ylabel('Firing Rate ECD');
            
            
            hold on;
            
            
            obj.figureHD = bar(downsample(obj.prefDirection180,obj.SAMPLE_RATE_HD), downsample(obj.hdCells,obj.SAMPLE_RATE_HD), 'b');
            sNeg = {sprintf('-180%c', char(176))};
                        sNeg= [sNeg,sprintf('-90%c', char(176))];
                        sNeg= [sNeg,sprintf('-0%c', char(176))];
                        sNeg= [sNeg,sprintf('90%c', char(176))];
                        sNeg= [sNeg,sprintf('180%c', char(176))];
                        set(gca,'FontSize',13,'FontWeight','bold');
                                                set(gca,'XTickLabel',sNeg);
                        set(gca,'XTick',[-180,-90,-0,90,180]);
            set(gca,'Box','off');   %# Turn off the box surrounding the whole axes
            axesPosition = get(gca,'Position');          %# Get the current axes position
            hNewAxes = axes('Position',axesPosition,...  %# Place a new axes on top...
                'Color','none',...           %#   ... with no background color           %#   ... and a different scale
                'YAxisLocation','right',...  %#   ... located on the right
                'XTick',[],...               %#   ... with no x tick marks
                'Box','off',...
                'YTickLabel',[]);                %#   ... and no surrounding box
            ylabel(hNewAxes,'Head Direction Activity');
            set(gca,'FontSize',13,'FontWeight','bold');
            
            obj.figureHD.XDataSource = 'X_HD';
            obj.figureHD.YDataSource = 'Y_HD';
            
            
            hold off;
            
            
            
            
            
        end
        
        function updatePlotCells(obj)
            % This function updates the plot
            
            X_HD = downsample(obj.prefDirection180,obj.SAMPLE_RATE_HD);
            Y_HD = downsample(obj.hdCells,obj.SAMPLE_RATE_HD);
            
            %             X_EGD = downsample(obj.prefDirection180,obj.SAMPLE_RATE_EGD);
            %             Y_EGD = downsample(obj.ecdCells,obj.SAMPLE_RATE_EGD);
            
            
            
            %             refreshdata(obj.figureEGD,'caller');
            refreshdata(obj.figureHD,'caller');
        end
        
    end
    
end

