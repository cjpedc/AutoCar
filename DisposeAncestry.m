function [] = DisposeAncestry(particleID)
% When the SLAM process is complete, this function will clean up the memory
% being used by the ancestry tree, and remove all of the associated entries
% from the low level map.

% TPath *tempPath, *trashPath;
% TEntryList *entry;

for i = 0 : ID_NUMBER
    if (particleID(i).ID == i)
        % Free up memory
        entry = particleID(i).mapEntries;
        for j = 0 : particleID(i).total
            LowDeleteObservation(entry(j).x, entry(j).y, entry(j).node);
        end
        free(entry);
        particleID(i).mapEntries = [];
        
        tempPath = particleID(i).path;
        while (~isempty(tempPath))
            trashPath = tempPath;
            tempPath = tempPath.next; % tempPath = tempPath->next;
            free(trashPath);
        end
        particleID(i).path = [];
        
        particleID(i).ID = -123;
    end
    
    for cleanID = 0 : ID_NUMBER
        availableID(cleanID) = cleanID;
    end
    cleanID = ID_NUMBER;
end
end

