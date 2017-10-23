
clear;
close all;

s = RandStream('mcg16807','Seed',25);
RandStream.setGlobalStream(s);

% size of the environment [x y]
environmentSize = [500, 500];
startPosition = [250 100];  

circledEnvironment = true;

runs = 200;

% number of cues
numPoints = 26;
angle = 2 * pi / numPoints;

pp = 0:angle:(2 * pi - angle);

% cues
ppx = cos(pp ).*300 + environmentSize(1)/2 ;
ppy = sin(pp ).*300 + environmentSize(2)/2 ;
cues = [ppx' , ppy'];

% goal location
goal =  cues(5,:);


% goal location
goalNumber = 4;
goalNumbers = [goalNumber,25,21,3,18,12,25];
episodes = length(goalNumbers);

% build environment model
environment = Environment(environmentSize,startPosition,cues(goalNumbers(1),:),cues,circledEnvironment);
% initilize figure plot
environment.initializeEnvironmentPlot();
% initilize figure plot
% build agent object
agent = Agent(startPosition,cues(goalNumbers(1),:),cues);
% set HD and use it to prepare and activate all cell layers
agent.initializeAgentCells();

agentPosition = startPosition;
agentOrientation = 0;


for k=1:episodes
    
    
    cueDistance = 65;
    
    for i=1:50000
        
        if goalNumbers(k) == 20
            disp('foo');
        end
        
        
        if pdist2(agentPosition,cues(goalNumbers(k),:)) < cueDistance
            break;
        end
        
        [movement,orientation] = agent.act(agentPosition,agentOrientation,cues(goalNumbers(k),:));
        [agentPosition,agentOrientation] = environment.update(movement,orientation);
        
        %%% Plotting stuff goes here %%%
        environment.updateFigure(cues(goalNumbers(k),:));
        
        
        
    end
end



