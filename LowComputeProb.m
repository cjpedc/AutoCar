function [unknown] = LowComputeProb(x, y, distance, ID)
% This function performs the exact same function as the one above, except
% that it can work without the observation array. This is useful for printing
% out the map or doing debugging or similar investigation in areas which
% are not within the area currently being observed.

if (isempty(lowMap(x,y)))
    unknown = UNKNOWN;
    return;
end

while (1)
    for i = 1 : lowMap(x,y).total % lowMap(x,y)->total
        if (lowMap(x,y).array(i).ID == ID) % lowMap(x,y)->array[i].ID
            if (lowMap(x,y).array(i).hits == 0) % lowMap(x,y)->array[i].hits
                unknown = 0;
                return;
            end
            unknown = (1.0 - exp(-(lowMap(x,y).array(i).hits...
                / lowMap(x,y).array(i).distance) *  distance));
            %lowMap(x,y)->array(i).hits/lowMap(x,y)->array(i).distance
            return;
        end
    end
    
    if (isempty(l_particleID(ID).parent))
        unknown = UNKNOWN;
        return;
    else
        ID = l_particleID(ID).parent.ID; % l_particleID[ID].parent->ID
    end
end
unknown = UNKNOWN;
return;

end

