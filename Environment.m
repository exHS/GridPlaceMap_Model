classdef Environment < handle
    %ENVIRONMENT This class simulates the environment
    %   Detailed explanation goes here
    
    properties(Access=private, Constant=true)
        % the default main orientation vector (O degrees)
        MAIN_ORIENTATION = [0 1];
        
        % the line length for orientation line
        LINE_LENGTH = 15;
        

    end
    
    properties(Access=private)
        
        % the height of the environment
        height = 0;
        % the width of the environment
        width = 0;
        % stores the current position of the agent [x,y]
        agentPosition = [0 0];
        % stores the current allocentric agent head direction in degrees
        agentOrientation = 0;
        % speed of the agent (default 10)
        agentVelocity = 10;
        % stores the current time
        currentTime = 0;
        % stores the complete path travelled so far
        travelledPath = [];
        % stores the figure variable
        pathFigure = [];

        % stores the goal direction
        goal = [];
        % stores the cues
        cues = [];
        % parameter to use circled environment
        circledEnvironment = [];
        
        % turning speed
        turningSpeed = 1.4;
        
        %%% Graphic Handles %%%
        % Handle the object, which draws the path graph.
        figurePathComplete;
        % Handles the line plot, which draws the orientation
        figureAgentOrientation;
        % Handles the scatter plot, which draws the end position.
        figurePathEnd;
        % Handles the scatter plot, which draws the goal position
        figureGoal;
        % Handles the scatter plot, which draws the cues
        figureCues;
    end
    
    properties(Access=private,Constant=true)
        % duration of one time step
        DELTA_T = 0.25;
        
        % DEBUG MOD
        DEBUG_MODE = 0;
    end
    
    methods(Access=public)
        
        function obj = Environment(environmentSize,agentStartPosition,agentVelocity,goal,cues,circledEnvironment)
            % Constructor of the environment class. It receives the size of the environment  [x,z] and agent information to initialize the environment
            % accordingly
            
            % set values
            obj.width = environmentSize(1);
            obj.height = environmentSize(2);
            obj.agentPosition = agentStartPosition;
            obj.travelledPath = agentStartPosition;
            obj.agentOrientation = obj.MAIN_ORIENTATION;
            obj.agentVelocity = agentVelocity;
            obj.goal = goal;
            obj.cues = cues;
            obj.circledEnvironment = circledEnvironment;
            % reset time
            obj.currentTime = 0.0;
        end
        
        
        function [agentPosition,agentOrientation] = update(obj, movementCommands, orientation)
            % This function receives egocentric movement and uses it to
            % calculate the new position (and orientation?) of the agent in
            % the environment
            
            % move the agent in the environment using the movement vectors and the orientation
            % calculate new orientation, using the motor commands to determine the turning speed
            orientation = mod(deg2rad(orientation) + tan(movementCommands(2)-movementCommands(1))* obj.DELTA_T * obj.turningSpeed,2*pi);
            
            % calculate new position using the calculated orientation and the predetermined velocity
            % of the agent
            obj.agentPosition(1) = obj.agentPosition(1) +  obj.DELTA_T * obj.agentVelocity * sin(orientation);
            obj.agentPosition(2) = obj.agentPosition(2) +  obj.DELTA_T * obj.agentVelocity * cos(orientation);
            
            % set agent orientation
            obj.agentOrientation = rad2deg(orientation);
            agentOrientation = obj.agentOrientation;
            
            % update the travelled path
            obj.travelledPath  = [obj.travelledPath ; obj.agentPosition];
            
            % set agentPosition
            agentPosition = obj.agentPosition;
            
            % upadte time
            obj.currentTime = obj.currentTime + obj.DELTA_T;
            
        end
        
        
        function initializeEnvironmentPlot(obj)
            % This functions depicts the enviroment with a path the agent
            % has travelled so far. Blue point is start position, red point
            % is end position
            obj.pathFigure =  figure('name','Traveled Path', 'OuterPosition',[0,0,1920,1080]);
            
            subplot(5,4,[1 2 5 6 9 10 13 14 17 18])
            
            hold on;
            axis square;
            
            X = obj.travelledPath(:,1);
            Y = obj.travelledPath(:,2);
            
            X_0 = obj.travelledPath(1,1);
            Y_0 = obj.travelledPath(1,2);
            
            X_n = obj.travelledPath(end,1);
            Y_n = obj.travelledPath(end,2);
            
            % Calculate line for orientation
            
            X_orientation = [ X_n , obj.MAIN_ORIENTATION(1) * obj.LINE_LENGTH + X_n ];
            Y_orientation = [ Y_n , obj.MAIN_ORIENTATION(2) * obj.LINE_LENGTH + Y_n ];
            
            
            % the orientation line of the agent
            obj.figureAgentOrientation = plot(X_orientation,Y_orientation,'red','LineWidth',1.5);
            obj.figureAgentOrientation.XDataSource = 'X_orientation';
            obj.figureAgentOrientation.YDataSource = 'Y_orientation';
            
            % the agent
            obj.figurePathEnd = scatter(X_n,Y_n,80,'red','filled');
            obj.figurePathEnd.XDataSource = 'X_n';
            obj.figurePathEnd.YDataSource = 'Y_n';            
            
            % the complete path
            obj.figurePathComplete = plot(X,Y,'-b');
            obj.figurePathComplete.XDataSource = 'X';
            obj.figurePathComplete.YDataSource = 'Y';
            
            % the start point
            scatter(X_0,Y_0,'blue');
          
            
            if obj.circledEnvironment
                
                % plot the border of the circle
                x = obj.width/2;
                y = obj.height/2;
                r = obj.width/1.75;
                ang=0:0.01:2*pi;
                xp=r*cos(ang);
                yp=r*sin(ang);
                plot(x+xp,y+yp,'black','LineWidth',2.0);

            end
                            
            % the cues
            numPoints = 4;
            angle = 2 * pi / numPoints;

            pp = 0:angle:(2 * pi - angle);
            ppx = cos(pp).*325 + obj.width/2 -12.5;
            ppy = sin(pp).*320 + obj.height/2;
            obj.figureCues = scatter(obj.cues(:,1),obj.cues(:,2),100,'filled');
            a = [1:length(obj.cues)]'; b = num2str(a); c = cellstr(b);
            text(obj.cues(:,1) + 4,obj.cues(:,2) + 4,c);
            obj.figureCues.XDataSource = 'X_cues';
            obj.figureCues.YDataSource = 'Y_cues';
            a = [1:numPoints]'; b = num2str(a); c = cellstr(b);
            s1 = sprintf('0%c', char(176));
            s2 = sprintf('90%c', char(176));
            s3 = sprintf('180%c', char(176));
            s4 = sprintf('270%c', char(176));
            t = text(ppx,ppy,{s2,s1,s4,s3},40);


            set(t,'FontSize',18);
            % the goal point
            obj.figureGoal = scatter(obj.goal(1),obj.goal(2),100,[0 0.5 0],'filled');
            obj.figureGoal.XDataSource = 'X_goal';
            obj.figureGoal.YDataSource = 'Y_goal';
            
            title('Environment')
            xlabel('x');
            ylabel('y');
            %legend('Traveled Path','Start Position','End Position')

            axis([0-0.2*obj.width, 1.2*obj.width, 0-0.2*obj.height, 1.2*obj.height]);
            
            set(gca,'FontSize',20,'FontWeight','bold');
            set(gca,'ytick',[]);
            set(gca,'yticklabel',[]);
            set(gca,'xtick',[]);
            set(gca,'xticklabel',[]);
            axes('xcolor', [1 1 1], 'ycolor', [1 1 1]);

            
            drawnow;
            
        end
        
        function updateFigure(obj,goal)
            % Update Environment updates the plots of the environment.
            
            X_goal = goal(1);
            Y_goal = goal(2);
            
            X_cues = obj.cues(:,1);
            Y_cues = obj.cues(:,2);
            
            X_n = obj.travelledPath(end,1);
            Y_n = obj.travelledPath(end,2);
            
            X = obj.travelledPath(:,1);
            Y = obj.travelledPath(:,2);
            
            % Calculate line for orientation
            X_orientation = sin(deg2rad(obj.agentOrientation)) * obj.LINE_LENGTH + X_n;
            Y_orientation = cos(deg2rad(obj.agentOrientation)) * obj.LINE_LENGTH + Y_n;
            
            X_orientation = [ X_n, X_orientation ];
            Y_orientation = [ Y_n, Y_orientation ];
            
            refreshdata(obj.figurePathComplete,'caller');
            refreshdata(obj.figurePathEnd,'caller');
            refreshdata(obj.figureAgentOrientation,'caller');
            refreshdata(obj.figureCues,'caller');
            refreshdata(obj.figureGoal,'caller');

            drawnow;
            
        end
        
    end
    
end

