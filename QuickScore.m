function [inline] = QuickScore(sense, index, sampleNum)
% Basically the same as CheckScore, except that the value returned is just 
% a heuristic approximation, based on a small distance near the percieved 
% endpoint of the scan, which is intended to be used to quickly cull out 
% bad samples, without having to evaluate the entire trace. The evaluation 
% area is currently set at 3.5 grid squares before the endpoint to 3 grid 
% squares past the endpoint. There is no special reason for these specific 
% values, if you want to change them.

distance = 0;
eval = 0;

if (sense(index).distance >= MAX_SENSE_RANGE)
    inline = 1; % return 1/True
end

distance = max(0, sense(index).distance - 3.5);
eval = LowLineTrace(int8((newSample(sampleNum).x + (cos(sense(index).theta ...
       + newSample(sampleNum).theta) * distance))), int8((newSample(sampleNum).y...
       + (sin(sense(index).theta + newSample(sampleNum).theta) * distance))), ...
       (sense(index).theta + newSample(sampleNum).theta), 3.5, ...
       l_particle(newSample(sampleNum).parent).ancestryNode.ID, 3); %ancestryNode->ID
inline = max(MAX_TRACE_ERROR, eval);

end

