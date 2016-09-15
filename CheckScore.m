function [inline] = CheckScore(sense, index, sampleNum)

% Determine the fitness of a laser endpoint
%
% sense- the set of all current laser observations
% index- the specific number of the laser observation that we are going to
% score
% sampleNum- the idnex into the sample array for the sample we are
% currently concerned with
%
% Returns the unnormalized posterior for current particle

MAP_SCALE = 35;
LOW_VARIANCE = 0.017 * MAP_SCALE*MAP_SCALE;
MAX_TRACE_ERROR = exp(-24.0/LOW_VARIANCE);

a = LowLineTrace(newSample(sampleNum).x, newSample(sampleNum).y, ...
    (sense(index).theta + newSample(sampleNum).theta), ...
    sense(index).distance, ...
    l_particle(newSample(sampleNum).parent).ancestryNode.ID, 0);

inline = max(MAX_TRACE_ERROR, a);
end

