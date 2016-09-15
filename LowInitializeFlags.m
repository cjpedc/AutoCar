function [] = LowInitializeFlags()

% Each entry observationArray corresponds to a single grid square in the
% global map. observationID is a count of how many of these entries there
% are. For each one of these entries, obsX/obsY represent its x,y coordinate
% in the global map. flagMap is an array the size of the global map, which
% gives a proper index into observationArray for that location. Therefore,
% we are resetting all non-zero entries of flagMap while resetting the arrays
% of obsX/obsY

while observation > 0
    observationID = observationID - 1;
    flagMap(obsX(observationID),obsY(observationID)) = 0;
    obsX(observationID) = 0;
    obsY(observationID) = 0;
end
observationID = 1;

end