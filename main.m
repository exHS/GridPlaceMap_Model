
clear;
close all;

FAST_PLOTTING = 1;

s = RandStream('mcg16807','Seed',25);
RandStream.setGlobalStream(s);

% size of the environment [x y]
environmentSize = [500, 500];
% start position of the agent
startPosition = [1 250];
% velocity of the agent
agentVelocity = 10;

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


% goal locations
goalNumbers = [10,5,11,4,12,3,13,2,14,1,15,26,16,25,17,24,18,23,19,22,20,21,26,2,25,3,24,4,23,5,22,6,21,7,20,8,19,9,18,10,17,11,16,12,15,13,25];
r = randi([1 26],1,1000);

% add more random gials
goalNumbers = cat(2,goalNumbers,randi([1 26],1,1000));

episodes = length(goalNumbers);

% build environment model
environment = Environment(environmentSize,startPosition,agentVelocity,cues(goalNumbers(1),:),cues,circledEnvironment);
% initilize figure plot
environment.initializeEnvironmentPlot();
% initilize figure plot
% build agent object
agent = Agent(startPosition,agentVelocity, cues(goalNumbers(1),:),cues);
% set HD and use it to prepare and activate all cell layers
agent.initializeAgentCells();

agentPosition = startPosition;
agentOrientation = 0;


for k=1:episodes
    
    
    cueDistance = 65;
    
    for i=1:50000
        
        
        
        if pdist2(agentPosition,cues(goalNumbers(k),:)) < cueDistance
            break;
        end
        
        [movement,orientation] = agent.act(agentPosition,agentOrientation,cues(goalNumbers(k),:));
        [agentPosition,agentOrientation] = environment.update(movement,orientation);
        
        %%% Plotting stuff goes here %%%
        if ~FAST_PLOTTING
            environment.updateFigure(cues(goalNumbers(k),:));
        end
        
    end
    if FAST_PLOTTING
        environment.updateFigure(cues(goalNumbers(k),:));
    end
    
end



