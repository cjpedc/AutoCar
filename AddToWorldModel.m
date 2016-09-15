function [] = AddToWorldModel(sense, particleNum)
% sense- the current laser observations
% particleNum - the index into the particle array to the entry that is
% making additions to the map

% Run through each point that the laser found an obstruction at

for j = 1:SENSE_NUMBER
    % Normalize readings relative to the pose of current assumed position
    LowAddTrace(l_particle(particleNum).x, l_particle(particleNum).y, ...
        sense(j).distance, (sense(j).theta + l_particle(particleNum).theta), ...
        l_particle(particleNum).ancestryNode.ID, ...
        (sense(j).distance < MAX_SENSE_RANGE));
end

end

