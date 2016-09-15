function [] = Localize(sense)
% This is where the bulk of evaluating and resampling the particles takes place.
% Also applies the motion model

PARTICLE_NUMBER = 50;
SAMPLE_NUMBER = PARTICLE_NUMBER * 10;
MAP_SCALE = 35;

ftemp = 0;
threshold = 0;  % threshhold for discarding particles (in log prob.)
total = 0;
[turn, distance, moveAngle] = deal(0); % The incremental motion reported by the odometer
[CCenter, DCenter, TCenter, CCoeff, DCoeff, TCoeff] = deal(0);
[tempC, tempD] = deal(0); % Temporary variables for the motion model.
[i, j, k, p, best] = deal(0); % Incremental counters.
keepers = 0; % How many particles finish all rounds
newchildren = [1:SAMPLE_NUMBER]; % Used for resampling

% Take the odometry readings from both this time step and the last, in order
% to figure out the base level of incremental motion. Convert our measurements
% from meters and degrees into terms of map squares and radians
distance = sqrt( ((odometry.x - lastX) * (odometry.x - lastX)) ...
    + ((odometry.y - lastY) * (odometry.y - lastY)) ) * MAP_SCALE;

turn = (odometry.theta - lastTheta);

% Keep motion bounded between pi and -pi
if (turn > pi/3)
    turn = turn - 2 * pi;
elseif (turn < -pi/3)
    turn = turn + 2*pi;
end

% Our motion model consists of motion along three variables; D is the major
% axis of motion, which is lateral motion along the robot's average facing
% angle for this time step, C is the minor axis of lateral motion, which is
% perpendicular to D, and T is the amount of turn in the robot's facing angle.
% Since the motion model is probablistic, the *Center terms compute the
% expected center of the distributions of C D and T. Note that these numbers
% are each a function of the reported odometric distance and turn which have
% been observed. The constant meanC_D is the amount of effect that the
% distance reported from the odometry has on our motion model's expected
% motion along the minor axis. All of these constants are defined at the
% top of this file.

CCenter = distance * meanC_D + turn * meanC_T;
DCenter = distance * meanD_D + turn * meanD_T;
TCenter = distance * meanT_D + turn * meanT_T;

% *Coeff computes the standard deviation for each parameter when generating
% gaussian noise. These numbers are limited to have at least some minimal
% level of noise, regardless of the reported motion. This is especially
% important for dealing with a robot skidding or sliding or just general
% unexpected motion which may not be reported at all by the odometry (it
% happens more often than we would like)

CCoeff = max((abs(distance*varC_D) + abs(turn*varC_T)), 0.8);
DCoeff = max((abs(distance*varD_D) + abs(turn*varD_T)), 0.8);
TCoeff = max((abs(distance*varT_D) + abs(turn*varT_T)), 0.10);

% To start this function, we have already determined which particles have
% been resampled, and how many times. What we still need to do is move them
% from their parent's position, according to the motion model, so that we
% have the appropriate scatter.
i = 0;
% Iterate through each of the old particles, to see how many times it got
% resampled.
for j = 0 : PARTICLE_NUMBER
    % Now create a new sample for each time this particle got resampled
    % (possibly 0)
    for k = 0 : children(j)
        % We make a sample entry. The first, most important value is which
        % of the old particles is this new sample's parent. This defines
        % which map is being inherited, which will be used during localization
        % to evaluate the "fitness" of that sample.
        newSample(i).parent = j;
        
        % Randomly calculate the 'probable' trajectory, based on the movement
        % model. The starting point is of course the position of the parent.
        tempC = CCenter + GAUSSIAN(CCoeff); % The amount of motion along
        % the minor axis of motion
        tempD = DCenter + GAUSSIAN(DCoeff); % The amount of motion along the
        % major axis of motion
        % Record this actual motion. If we are using hierarchical SLAM, it
        % will be used to keep track of the "corrected" motion of the robot,
        % to define this step of the path.
        newSample(i).C = tempC;
        newSample(i).D = tempD;
        newSample(i).T = TCenter + GAUSSIAN(TCoeff);
        newSample(i).theta = l_particle(j).theta + newSample(i).T;
        
        % Assuming that the robot turned continuously throughout the time
        % step, the major direction of movement (D) should be the average
        % of the starting angle and the final angle
        moveAngle = (newSample(i).theta + l_particle(j).theta)/2.0;
        
        % The first term is to correct for the LRF not being mounted on the
        % pivot point of the robot's turns The second term is to allow for
        % movement along the major axis of movement (D)
        % The last term is movement perpendicular to the the major axis (C).
        % We add pi/2 to give a consistent "positive" direction for this
        % term. MeanC significantly shifted from 0 would mean that the robot
        % has a distinct drift to one side.
        newSample(i).x = l_particle(j).x + (TURN_RADIUS * (cos(newSample(i).theta)...
            - cos(l_particle(j).theta))) + (tempD * cos(moveAngle)) + ...
            (tempC * cos(moveAngle + pi/2));
        newSample(i).y = l_particle(j).y + (TURN_RADIUS * (sin(newSample(i).theta)...
            - sin(l_particle(j).theta))) + (tempD * sin(moveAngle)) + ...
            (tempC * sin(moveAngle + pi/2));
        newSample(i).probability = 0.0;
        i = i + 1;
    end
end

% Go through these particles in a number of passes, in order to find the best
% particles. This is where we cull out obviously bad particles, by performing
% evaluation in a number of distinct steps. At the end of each pass, we
% identify the probability of the most likely sample. Any sample which is not
% within the defined threshhold of that probability can be removed, and no
% longer evaluated, since the probability of that sample ever becoming "good"
% enough to be resampled is negligable.
% Note: this first evaluation is based solely off of QuickScore- that is,
% the evaluation is only performed for a short section of the laser trace,
% centered on the observed endpoint. This can provide a good, quick heuristic
% for culling off bad samples, but should not be used for final weights.
% Something which looks good in this scan can very easily turn out to be low
% probability when the entire laser trace is considered.

threshold = WORST_POSSIBLE-1; % ensures that we accept anything in 1st round
for p = 0 : PASSES
    best = 0;
    for i = i : SAMPLE_NUMBER
        if (newSample(i).probability >= threshold)
            for k = p :p + PASSES :SENSE_NUMBER % (k = p; k < SENSE_NUMBER; k += PASSES)
                newSample(i).probability = newSample(i).probability + ...
                    log(QuickScore(sense, k, i));
            end
            if (newSample(i).probability > newSample(best).probability)
                best = i;
            end
        else
            newSample(i).probability = WORST_POSSIBLE;
        end
    end
    threshold = newSample(best).probability - THRESH;
end

keepers = 0;
for i = 0 : SAMPLE_NUMBER
    if (newSample(i).probability >= threshold)
        keepers = keepers + 1;
        % Don't let this heuristic evaluation be included in the final eval.
        newSample(i).probability = 0.0;
    else
        newSample(i).probability = WORST_POSSIBLE;
    end
end

% Letting the user know how many samples survived this first cut.
sprintf('Better %d', keepers);
threshold = -1;

% Now reevaluate all of the surviving samples, using the full laser scan to
% look for possible obstructions, in order to get the most accurate weights.
% While doing this evaluation, we can still keep our eye out for unlikely
% samples before we are finished.
keepers = 0;
for p = 0 : PASSES
    best = 0;
    for i = 0 : SAMPLE_NUMBER
        if (newSample(i).probability >= threshold)
            if (p == PASSES -1)
                keepers = keepers + 1;
            end
            for k = p : p+PASSES : SENSE_NUMBER % (k = p; k < SENSE_NUMBER; k += PASSES)
                newSample(i).probability = newSample(i).probability + ...
                    log(CheckScore(sense, k, i));
            end
            if (newSample(i).probability > newSample(best).probability)
                best = i;
            end
        else
            newSample(i).probability = WORST_POSSIBLE;
        end
    end
    threshold = newSample(best).probability - THRESH;
end

% Report how many samples survived the second cut. These numbers help the
% user have confidence that the threshhold values used for culling are
% reasonable.
sprintf('Best of %d', keepers);

% All probabilities are currently in log form. Exponentiate them, but weight
% them by the prob of the the most likely sample, to ensure that we don't
% run into issues of machine precision at really small numbers.

total = 0.0;
threshold = newSample(best).probability;
for i = 0 : SAMPLE_NUMBER
    % If the sample was culled, it has a weight of 0
    if (newSample(i).probability == WORST_POSSIBLE)
        newSample(i).probability = 0.0;
    else
        newSample(i).probability = exp(newSample(i).probability-threshold);
        total = total + newSample(i).probability;
    end
end

% Renormalize to ensure that the total probability is now equal to 1.
for i = 0 : SAMPLE_NUMBER
    newSample(i).probability = newSample(i).probability/total;
end

total = 0.0;
% Count how many children each particle will get in next generation
% This is done through random resampling.
for i = 0 : SAMPLE_NUMBER
    newchildren(i) = 0;
    total = total + newSample(i).probability;
end

[i j] = deal(0); % i = no. of survivors, j = no. of new samples
while ((j < SAMPLE_NUMBER) && (i < PARTICLE_NUMBER))
    k = 0;
    ftemp = MTrandDec()*total;
    while (ftemp > (newSample(k).probability))
        ftemp = ftemp - newSample(k).probability;
        k = k + 1;
    end
    if (newchildren(k) == 0)
        i = i + 1;
    end
    newchildren(k) = newchildren(k) + 1;
    j = j + 1;
end

% Report exactly how many samples are kept as particles, since they were
% actually resampled.
sprintf('(%d kept)', i);

% Do some cleaning up
% Is this even necessary?
for i = 0 : PARTICLE_NUMBER
    children(i) = 0;
    savedParticle(i).probability = 0.0;
end

% Now copy over new particles to savedParticles
best = 0;
k = 0; % pointer into saved particles
for i = 0 : SAMPLE_NUMBER
    if (newchildren(i) > 0) {
        savedParticle(k).probability = newSample(i).probability;
        savedParticle(k).x = newSample(i).x;
        savedParticle(k).y = newSample(i).y;
        savedParticle(k).theta = newSample(i).theta;
        savedParticle(k).C = newSample(i).C;
        savedParticle(k).D = newSample(i).D;
        savedParticle(k).T = newSample(i).T;
        % For savedParticle, the ancestryNode field actually points to the
        % parent of this saved particle
        savedParticle(k).ancestryNode = l_particle( newSample(i).parent ).ancestryNode;
        % savedParticle[k].ancestryNode->numChildren++;
        savedParticle(k).ancestryNode.numChildren = savedParticle(k).ancestryNode.numChildren + 1; ;
        children(k) = newchildren(i);
        
        if (savedParticle(k).probability > savedParticle(best).probability)
            best = k;
        end
        k = k + 1;
    end
end

% This number records how many saved particles we are currently using, so
% that we can ignore anything beyond this in later computations.
cur_saved_particles_used = k;

% We might need to continue generating children for particles, if we reach
% PARTICLE_NUMBER worth of distinct parents early
% We renormalize over the chosen particles, and continue to sample from there.

if (j < SAMPLE_NUMBER)
    total = 0.0;
    % Normalize particle probabilities. Note that they have already been exponentiated
    for i = 0 : cur_saved_particles_used
        total = total + savedParticle(i).probability;
    end
    for i = 0 : cur_saved_particles_used
        savedParticle(i).probability = savedParticle(i).probability/total;
    end
    total = 0.0;
    for i = 0 : cur_saved_particles_used
        total = total + savedParticle(i).probability;
    end
    
    while (j < SAMPLE_NUMBER)
        k = 0;
        ftemp = MTrandDec()*total; % MTrandDec()???
        while (ftemp > (savedParticle(k).probability))
            ftemp = ftemp - savedParticle(k).probability;
            k = k + 1;
        end
        children(k) = children(k) + 1;
        
        j = j + 1;
    end
end

% Some useful information concerning the current generation of particles,...
% and the parameters for the best one.
sprintf('-- %.3d (%.4f, %.4f, %.4f) : %.4f\n', curGeneration, ...
    savedParticle(best).x, savedParticle(best).y, savedParticle(best).theta,...
    savedParticle(best).probability);

end

