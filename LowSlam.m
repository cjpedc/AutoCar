function [] = LowSlam(continueSlam, path, obs)
% The main function for performing SLAM at the low level. The first argument
% will return whether there is still information to be processed by SLAM
% (set to 1). The second and third arguments return the corrected odometry
% for the time steps, and the corresponding list of observations. This can
% be used for the higher level SLAM process when using hierarchical SLAM.

moveAngle = 0;
counter = 0;
overflow = 0;
name = zeros(1, 32);
tempPath = 0; % struct and pointer
tempObs = 0; % struct and pointer
lineage = 0; % struct and pointer

% We need to know, for various purposes, how big our map is allowed to be
MAP_WIDTH = 1700;
MAP_HEIGHT = 1700;

% This is the number of particles that we are keeping at the low level
PARTICLE_NUMBER = 50;

% This is the number of samples that we will generate each iteration.
% Notice that we generate more samples than we will keep as actual
% particles. This is because so many are "bad" samples, and clearly won't
% be resampled, thus we don't need to allocate nearly as much memory if we
% acknowledge that a certain amount will never be considered particles.
% "Localize" function in low.m can help explain.
SAMPLE_NUMBER = (PARTICLE_NUMBER * 10);

% When using hierarchical SLAM, LOW_DURATION defines how many iterations of
% the low level are performed to form one iteration of the high level. Setting this value to ridiculously high
% values can essentially turn off the hierarchical feature.
LOW_DURATION = 40;

% Initialize the worldMap
LowInitializeWorldMap();

% Initialize the ancestry and particles
ID_NUMBER = ceil(PARTICLE_NUMBER * 2.25);
cleanID = ID_NUMBER - 2; % ID_NUMBER-1 is being used as the root of the 
                         % ancestry tree.

% Initialize all of our unused ancestor particles to look unused.
for i = 1:ID_NUMBER
    availableID(i) = i;
    
    l_particleID(i).generation = -1;
    l_particleID(i).generation = -1;
    l_particleID(i).numChildren = 0;
    l_particleID(i).ID = -1;
    l_particleID(i).parent = 0; % null
    l_particleID(i).mapEntries = 0; % null
    l_particleID(i).path = 0; % null
    l_particleID(i).seen = 0;
    l_particleID(i).total = 0;
    l_particleID(i).size = 0;
    
end

% Initialize the root of our ancestry tree.
l_particleID(ID_NUMBER-1).generation = 0;
l_particleID(ID_NUMBER-1).numChildren = 1;
l_particleID(ID_NUMBER-1).size = 0;
l_particleID(ID_NUMBER-1).total = 0;
l_particleID(ID_NUMBER-1).ID = ID_NUMBER-1;
l_particleID(ID_NUMBER-1).parent = 0; % null
l_particleID(ID_NUMBER-1).mapEntries = 0; % null

% Create all of our starting particles at the center of the map.
for i = 1:PARTICLE_NUMBER
    l_particle(i).ancestryNode = (l_particleID(ID_NUMBER-1)); % address &(l_particleID(ID_NUMBER-1))
    l_particle(i).x = MAP_WIDTH / 2;
    l_particle(i).y = MAP_HEIGHT / 2;
    l_particle(i).theta = 0.001;
    l_particle(i).probability = 0;
    children(i) = 0;
end

% We really only use the first particle, since they are all essentially the same.
l_particle(1).probability = 1;
l_cur_particles_used = 1;
children(1) = SAMPLE_NUMBER;

% We don't need to initialize the savedParticles, since Localization will
% create them for us, and they first are used in UpdateAncestry, which is
% called after Localization. This statement isn't necessary, then, but serves
% as a sort of placeholder when reading the code.
cur_saved_particles_used = 0;

% Make a record of what the first odometry readings were, so that we can
% compute relative movement across time steps.
lastX = odometry.x;
lastY = odometry.y;
lastTheta = odometry.theta;

overflow = 1;

% Add the first thing that you see to the worldMap at the center. This gives
% us something to localize off of.
if curGeneration == 0
    AddToWorldModel(sense, 0);
    for i = 1:SENSE_NUMBER
        hold(1).sense(i).distance = sense(i).distance;
        hold(1).sense(i).theta = sense(i).theta;
    end
    curGeneration = curGeneration + 1;
else
    
    LowInitializeFlags();
    % Add our first observation to our map of the world. This will serve as the basis for future localizations
    AddToWorldModel(hold(int8(LOW_DURATION * 0.5)).sense, 0);
    for i = (int8(LOW_DURATION * 0.5 + 1)) : LOW_DURATION 
        LowInitializeFlags();
        % Move the particles one step
        moveAngle = l_particle(1).theta + (hold(i).T/2.0);
        
        l_particle(1).x = l_particle(1).x + (TURN_RADIUS * (cos(l_particle(1).theta + hold(i).T) ...
            - cos(l_particle(1).theta))) + (hold(i).D * cos(moveAngle)) + ...
            (hold(i).C * cos(moveAngle + pi/2));
        
        l_particle(1).y = l_particle(1).y + (TURN_RADIUS * (sin(l_particle(1).theta + hold(i).T) ...
            - sin(l_particle(1).theta))) + (hold(i).D * sin(moveAngle)) + ...
            (hold(i).C * sin(moveAngle + pi/2));
        
        l_particle(1).theta = l_particle(1).theta + hold(i).T;
        
        AddToWorldModel(hold(i).sense, 0);
    end
    
    for i = 1 : SENSE_NUMBER
        hold(1).sense(i).distance = hold(LOW_DURATION-1).sense(i).distance;
        hold(1).sense(i).theta = hold(LOW_DURATION-1).sense(i).theta;
    end
    
end

% Get our observation log started.
% obs = (TSenseLog *)malloc(sizeof(TSenseLog));
for i = 1 : SENSE_NUMBER
    %(*obs)->sense(i).distance = hold(0).sense(i).distance;
    obs.sense(i).distance = hold(1).sense(i).distance;
    %(*obs)->sense(i).theta = hold(0).sense(i).theta;
    obs.sense(i).theta = hold(1).sense(i).theta;
end
%(*obs)->next = NULL;
obs.next = 0; %null %next pointer has to be modified

continueSlam = 1;
counter = 0;

while continueSlam && (counter < LOW_DURATION)
    % We take our readings now, because we are on the move, and they will likely
    % change while this procedure is running. This way we have them coordinated.
    if PLAYBACK == ''
        GetOdometry(odometry);
        % If there is no command to move, only localize one more time step, to account
        % for lag between a command to stop and SLAM completing an iteration. Otherwise,
        % without a command to move, we will assume that there is no (significant) movement
        % being made
        % This has been disabled for playback logs. It will be used again when we
        % use the software on the robot directly.
        % if ( (TranslationSpeed != 0.0) || (RotationSpeed != 0.0) )
        %   overflow = 2;
        % end
    else
        % Collect information from the data log. If either reading returns
        % 1, we've run out of log data, and we need to stop now.
        if ((ReadLog(readFile, sense, continueSlam) == 1) || (ReadLog(readFile, sense, continueSlam) == 1))
            overflow = 0;
        else
            overflow = 1;
        end
    end
    
    % We don't necessarily want to use every last reading that comes in.
    % This allows us to make certain that the robot has moved at least a
    % minimal amount (in terms of meters and radians) before we try to
    % localize and update.
    if ((sqrt(SQUARE(odometry.x - lastX) + SQUARE(odometry.y - lastY)) < 0.05) ...
            && (abs(odometry.theta - lastTheta) < 0.03))
        overflow = 0;
    end
    
    if (overflow > 0)
        overflow = overflow - 1;
        
        % Record and preprocess the current laser reading
        if (PLAYBACK == '')
            GetSensation(sense);
        end
        
        % Wipe the slate clean
        LowInitializeFlags();
        
        % Apply the localization procedure, which will give us the N best particles
        Localize(sense);
        
        % Add these maintained particles to the FamilyTree, so that ancestry can be determined, and then prune dead lineages
        UpdateAncestry(sense, l_particleID);
        
        % Update the observation log (used only by hierarchical SLAM)
        tempObs = obs; % tempObs = (*obs);
        while (~isempty(tempObs.next)) %next pointer has to be modified
            tempObs = tempObs.next; %next pointer has to be modified
        end
        % tempObs.next = (TSenseLog *)malloc(sizeof(TSenseLog));
        tempObs.next = TSenseLog; %next pointer has to be modified
        if (isempty(tempObs.next)) %next pointer has to be modified
            disp('Malloc failed in making a new observation!\n');
        end
        for i = 1 : SENSE_NUMBER
            tempObs.next.sense(i).distance = sense(i).distance; %next pointer has to be modified
            tempObs.next.sense(i).theta = sense(i).theta; %next pointer has to be modified
        end
        tempObs.next.next = []; %next pointer has to be modified
        
        % Holding Pen for observations.
        for i = 1 : SENSE_NUMBER
            hold(counter).sense(i).distance = sense(i).distance;
            hold(counter).sense(i).theta = sense(i).theta;
        end
        
        curGeneration = curGeneration + 1;
        counter = counter + 1;
        
        % Remember these odometry readings for next time. This is what lets us know the incremental motion.
        lastX = odometry.x;
        lastY = odometry.y;
        lastTheta = odometry.theta;
    end
end

% Find the most likely particle. Return its path
% Used only by Hierarchical SLAM
j = 0;
for i = 1 : l_cur_particles_used
    if (l_particle(i).probability > l_particle(j).probability)
        j = i;
    end
end

% (*path) = NULL;
path = [];
i = 0;
lineage = l_particle(j).ancestryNode;
while ((~isempty(lineage)) && (lineage.ID ~= ID_NUMBER-1))
    tempPath = lineage.path;
    i = i + 1;
    while (~isempty(tempPath.next)) %next pointer has to be modified
        i = i + 1;
        tempPath = tempPath.next; %next pointer has to be modified
    end
    tempPath.next = path; %next pointer has to be modified
    
    path = lineage.path;
    lineage.path = [];
    lineage = lineage.parent;
end

tempPath = path;
i = 0;
while (~isempty(tempPath))
    hold(i).C = tempPath.C;
    hold(i).D = tempPath.D;
    hold(i).T = tempPath.T;
    tempPath = tempPath.next; %next pointer has to be modified
    i = i + 1;
end

% Print out the map. PLOT
% sprintf(name, "lmap%.2d", (int) (curGeneration/LOW_DURATION)-1);
j = 0;
for i = 1 : l_cur_particles_used + 1
    if (l_particle(i).probability > l_particle(j).probability)
        j = i;
    end
end
%PrintMap(name, l_particle[j].ancestryNode, FALSE, -1, -1, -1);
%sprintf(name, "rm lmap%.2d.ppm", (int) (curGeneration/LOW_DURATION)-1);
%system(name);

% Clean up the memory being used.
DisposeAncestry(l_particleID);
LowDestroyMap();

end

