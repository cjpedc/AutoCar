function [] = UpdateAncestry(sense, particleID)
% Every iteration, after particles have been resampled and evaluated (ie
% after Localize has been run), we will need to update the ancestry tree.
% This consists of four main steps.
% a) Remove dead nodes. These are defined as any ancestor node which has no
%    descendents in the current generation. This is caused by certain particles
%    not being resampled, or by a node's children all dying off on their own.
%    These nodes not only need to be removed, but every one of their observations
%    in their observations in the map also need to be removed.
% b) Collapse branches of the tree. We want to restrict each internal node
%    to having a branching factor of at least two. Therefore, if a node has
%    only one child, we merge the information in that node with the one child
%    node. This is especially common at the root of the tree, as the different
%    hypotheses coalesce.
% c) Add the new particles into the tree.
% d) Update the map for each new particle. We needed to wait until they
%    were added into the tree, so that the ancestry tree can keep track of
%    the different observations associated with the particle.

%   TAncestor *temp, *hold, *parentNode;
%   TEntryList *entry, *workArray;
%   TMapStarter *node;
%   TPath *tempPath, *trashPath;

% Remove Dead Nodes -
% Go through the current particle array, and prune out all particles that
% did not spawn any particles. We know that the particle had to spawn
% samples, but those samples may not have become particles themselves,
% through not generating any samples for the next generation. Recurse up
% through there.
for i = 1 : l_cur_particles_used
    temp = l_particle(i).ancestryNode;
    
    % This is a "while" loop for purposes of recursing up the tree.
    while (temp.numChildren == 0) % temp->numChildren
        % Free up the memory in the map by deleting the associated observations.
        for j = 1 : temp.total % temp->total
            % LowDeleteObservation(temp->mapEntries[j].x, temp->mapEntries[j].y, temp->mapEntries[j].node);
            LowDeleteObservation(temp.mapEntries(j).x, temp.mapEntries(j).y,...
                temp.mapEntries(j).node);
        end
        
        % Get rid of the memory being used by this ancestor
        free(temp.mapEntries); % free(temp->mapEntries);
        temp.mapEntries = [];
        
        % This is used exclusively for the low level in hierarchical SLAM.
        % Get rid of the hypothesized path that this ancestor used.
        tempPath = temp.path; % tempPath = temp->path;
        while (~isempty(tempPath))
            trashPath = tempPath;
            tempPath = tempPath.next; % tempPath->next
            free(trashPath); % free(trashPath);
        end
        temp.path = []; % temp->path = NULL;
        
        % Recover the ID, so that it can be used later.
        cleanID = cleanID + 1;
        availableID(cleanID) = temp.ID; % temp->ID;
        temp.generation = curGeneration; % temp->generation = curGeneration;
        temp.ID = -42; % temp->ID = -42;
        
        % Remove this node from the tree, while keeping track of its parent.
        % We need that for recursing up the tree (its parent could possibly
        % need to be removed itself, now)
        hold = temp;
        temp = temp.parent; % temp = temp->parent;
        hold.parent = NULL;% hold->parent = NULL;
        
        % Note the disappearance of this particle (may cause telescoping of
        % particles, or outright deletion)
        temp.numChildren = temp.numChildren - 1; % temp->numChildren--;
    end
end

% Collapse Branches -
% Run through the particle IDs, checking for those node IDs that are currently
% in use. Essentially is an easy way to pass through the entire tree.  If
% the node is in use, look at its parent. If the parent has only one child,
% give all of the altered map squares of the parent to the child, and dispose
% of the parent. This collapses those ancestor nodes which have a branching
% factor of only one.

for i = 1 : ID_NUMBER
    % These booleans mean (in order) that the ID is in use, it has a parent
    % (ie is not the root of the ancestry tree), and that its parent has only
    % one child (which is necessarily this ID)
    if ((particleID(i).ID == i) && ~isempty(particleID(i).parent) && ...
            (particleID(i).parent.numChildren == 1)) % particleID(i).parent->numChildren
        % This is a special value for redirections. If the node's parent has
        % already participated in a collapse during this generation, then that
        % node has already been removed from the tree, and we need to go up
        % one more step in the tree.
        while (particleID(i).parent.generation == -111) % particleID(i).parent->generation
            particleID(i).parent = particleID(i).parent.parent; %particleID(i).parent->parent;
        end
        parentNode = particleID(i).parent;
        
        % Check to make sure that the parent's array is large enough to
        % accomadate all of the entries of the child in addition to its own.
        % If not, we need to increase the dynamic array.
        
        % if (parentNode->size < (parentNode->total + particleID[i].total)) {
        % parentNode->size = (int)(ceil((parentNode->size + particleID[i].size)*1.75));
        % workArray = (TEntryList *)malloc(sizeof(TEntryList)*parentNode->size);
        if (parentNode.size < (parentNode.total + particleID(i).total))
            parentNode.size = int8(ceil((parentNode.size + particleID(i).size)*1.75));
            
            % workArray = (TEntryList *)malloc(sizeof(TEntryList)*parentNode->size);
            workArray = [];
            %if (workArray == NULL) fprintf(stderr, "Malloc failed for workArray\n");
            
            for j = 1 : parentNode.total % parentNode->total
                workArray(j).x = parentNode.mapEntries(j).x; % parentNode->mapEntries[j].x
                workArray(j).y = parentNode.mapEntries(j).y; % parentNode->mapEntries[j].y
                workArray(j).node = parentNode.mapEntries(j).node; %parentNode->mapEntries[j].node
            end
            % Note that parentNode->total hasn't changed- that will grow as the
            % child's entries are added in
            free(parentNode.mapEntries); % free(parentNode->mapEntries);
            parentNode.mapEntries = workArray; % parentNode->mapEntries
        end
        
        % Change all map entries of the parent to have the ID of the child
        % Also check to see if this entry supercedes an entry currently attributed
        % to the parent. Since collapses can merge all of the entries between the
        % parent and the current child into the parent, this check is performed
        % by comparing to see if the generation of the last observation (before the
        % child's update) is at least as recent as parent's generation. If so, note
        % that there is another "dead" entry in the observation array. It will be
        % cleaned up later. If this puts the total number of used slot, minus the
        % number of "dead", below the threshold, shrink the array (which cleans up
        % the dead)
        entry = particleID(i).mapEntries;
        for j = 1 : particleID(i).total
            node = lowMap(entry(j).x, entry(j).y);
            
            % Change the ID
            % node->array[entry[j].node].ID = parentNode->ID;
            node.array(entry(j).node).ID = parentNode.ID;
            % node->array[entry[j].node].source = parentNode->total;
            node.array(entry(j).node).source = parentNode.total;
            
            % 	parentNode->mapEntries[parentNode->total].x = entry[j].x;
            % 	parentNode->mapEntries[parentNode->total].y = entry[j].y;
            % 	parentNode->mapEntries[parentNode->total].node = entry[j].node;
            % 	parentNode->total++;
            
            parentNode.mapEntries(parentNode.total).x = entry(j).x;
            parentNode.mapEntries(parentNode.total).y = entry(j).y;
            parentNode.mapEntries(parentNode.total).node = entry(j).node;
            parentNode.total = parentNode.total + 1;
            
            %  Check for pre-existing observation in the parent's list
            %     if (node->array[entry[j].node].parentGen >= parentNode->generation)
            % 	  node->array[entry[j].node].parentGen = -1;
            % 	  node->dead++;
            if (node.array(entry(j).node).parentGen >= parentNode.generation)
                node.array(entry(j).node).parentGen = -1;
                node.dead = node.dead + 1;
            end
        end
        % We do this in a second pass for a good reason. If there are more than one
        % update for a given grid square which uses the child's ID (as a consequence
        % of an earlier collapse), then we want to make certain that the resizing
        % doesn't take place until after all entries have changed their ID appropriately.
        for j = 1 : particleID(i).total
            node = lowMap(entry(j).x,entry(j).y);
            % ((node->total - node->dead)*2.5 < node->size)
            if ((node.total - node.dead)*2.5 < node.size)
                LowResizeArray(node, -7);
            end
        end
        
        %  We're done with it- remove the array of updates from the child.
        free(particleID(i).mapEntries);
        particleID(i).mapEntries = [];
        
        % Inherit the path
        tempPath = parentNode.path; % parentNode->path
        while (~isempty(tempPath.next)) % tempPath->next != NULL
            tempPath = tempPath.next; % tempPath->next
        end
        tempPath.next = particleID(i).path;
        particleID(i).path = [];
        
        % Inherit the number of children
        % parentNode->numChildren = particleID[i].numChildren;
        parentNode.numChildren = particleID(i).numChildren;
        
        % Subtlety of the ancestry tree: since we only keep pointers up to the
        % parent, we can't exactly change all of the descendents of the child
        % to now point to the parent. What we can do, however, is mark the
        % change for later, and update all of the ancestor particles in a single
        % go, later. That will take a single O(P) pass
        particleID(i).generation = -111;
    end
end

% This is the step where we correct for redirections that arise from the
% collapse of a branch of the ancestry tree
for i = 1 : ID_NUMBER
    if (particleID(i).ID == i)
        while (particleID(i).parent.generation == -111) % particleID(i).parent->generation
            particleID(i).parent = particleID(i).parent.parent; % particleID(i).parent->parent;
        end
    end
end

% Wipe the slate clean, so that we don't get confused by the mechinations
% of the previous changes from deletes and merges. Updates can make thier
% own tables, as needed.
LowInitializeFlags();

% Add the current savedParticles into the ancestry tree, and copy them over
% into the 'real' particle array
j = 0;
for i = 1 : cur_saved_particles_used
    % Check for redirection of parent pointers due to collapsing of branches
    %(see above)
    while (savedParticle(i).ancestryNode.generation == -111) % savedParticle(i).ancestryNode->generation == -111)
        savedParticle(i).ancestryNode = savedParticle(i).ancestryNode.parent; % savedParticle(i).ancestryNode->parent;
    end
    % A saved particle has ancestryNode denote the parent particle for that saved
    % particle If that parent has only this one child, due to resampling, then
    % we want to perform a collapse of the branch, but it hasn't been done yet,
    % because the savedParticle hasn't been entered into the tree yet. Therefore,
    % we just designate the parent as the "new" entry for this savedParticle.
    % We then update the already created ancestry node as if it were the new node.
    
    if (savedParticle(i).ancestryNode.numChildren == 1) % savedParticle(i).ancestryNode->numChildren == 1
        % Change the generation of the node
        % savedParticle[i].ancestryNode->generation = curGeneration;
        savedParticle(i).ancestryNode.generation = curGeneration;
        % Now that it represents the new node as well, it no longer is considered
        % to have children.
        % savedParticle[i].ancestryNode->numChildren = 0;
        savedParticle(i).ancestryNode.numChildren = 0;
        
        % We're copying the savedParticles to the main particle array
        l_particle(j).ancestryNode = savedParticle(i).ancestryNode;
        
        % Add a new entry to the path of the ancestor node.
        % trashPath = (TPath *)malloc(sizeof(TPath));
        % trashPath->C = savedParticle[i].C;
        % trashPath->D = savedParticle[i].D;
        % trashPath->T = savedParticle[i].T;
        % trashPath->next = NULL;
        % tempPath = l_particle[i].ancestryNode->path;
        trashPath.C = savedParticle(i).C;
        trashPath.D = savedParticle(i).D;
        trashPath.T = savedParticle(i).T;
        trashPath.next = [];
        tempPath = l_particle(i).ancestryNode.path;
        while ~isempty(tempPath.next) % (tempPath->next != NULL)
            tempPath = tempPath.next; % tempPath = tempPath->next;
        end
        tempPath.next = trashPath; % tempPath->next = trashPath;
        
        l_particle(j).x = savedParticle(i).x;
        l_particle(j).y = savedParticle(i).y;
        l_particle(j).theta = savedParticle(i).theta;
        l_particle(j).probability = savedParticle(i).probability;
        j = j + 1;
        
        
        % IF the parent has multiple children, then each child needs its own new ancestor node in the tree
        % elseif (savedParticle[i].ancestryNode->numChildren > 0) {
    elseif (savedParticle(i).ancestryNode.numChildren > 0) {
        % Find a new entry in the array of ancestor nodes. This is done by
        % taking an unused ID off of the stack, and using that slot.
        % temp = &(particleID[ availableID[cleanID] ]);
        temp = (particleID( availableID(cleanID) ));
        temp.ID = availableID(cleanID); % temp->ID
        % That ID on the top of the stack is now being used.
        cleanID = cleanID - 1;
        
        if (cleanID < 0)
            sprintf(' !!! Insufficient Number of Particle IDs : Abandon Ship !!!\n')
            cleanID = 0;
        end
        
        % This new node needs to have its info filled in
        % temp->parent = savedParticle[i].ancestryNode;
        temp.parent = savedParticle(i).ancestryNode;
        % No updates to the map have been made yet for this node
        %       temp->mapEntries = NULL;
        %       temp->total = 0;
        %       temp->size = 0;
        temp.mapEntries = [];
        temp.total = 0;
        temp.size = 0;
        % The generation of this node is important for collapsing branches of the tree. See above.
        %       temp->generation = curGeneration;
        %       temp->numChildren = 0;
        %       temp->seen = 0;
        temp.generation = curGeneration;
        temp.numChildren = 0;
        temp.seen = 0;
        
        % This is where we add a new entry to this node's hypothesized path for the robot
        % trashPath = (TPath *)malloc(sizeof(TPath));
        % trashPath->C = savedParticle[i].C;
        % trashPath->D = savedParticle[i].D;
        % trashPath->T = savedParticle[i].T;
        % trashPath->next = NULL;
        % temp->path = trashPath;
        trashPath.C = savedParticle(i).C;
        trashPath.D = savedParticle(i).D;
        trashPath.T = savedParticle(i).T;
        trashPath.next = [];
        temp.path = trashPath;
        
        % Transfer this entry over to the main particle array
        l_particle(j).ancestryNode = temp;
        l_particle(j).x = savedParticle(i).x;
        l_particle(j).y = savedParticle(i).y;
        l_particle(j).theta = savedParticle(i).theta;
        l_particle(j).probability = savedParticle(i).probability;
        j = j + 1;
    end
end

l_cur_particles_used = cur_saved_particles_used;

% Here's where we actually go through and update the map for each particle.
% We had to wait until now, so that the appropriate structures in the
% ancestry had been created and updated.
for i = 1 : l_cur_particles_used
    AddToWorldModel(sense, i);
end

% Clean up the ancestry particles which disappeared in branch collapses.
% Also, recover their IDs. We waited until now because we needed to allow
% for redirection of parents.
for i = 1 : ID_NUMBER-1
    if (particleID(i).generation == -111)
        particleID(i).generation = -1;
        particleID(i).numChildren = 0;
        particleID(i).parent = [];
        particleID(i).mapEntries = [];
        particleID(i).path = [];
        particleID(i).seen = 0;
        particleID(i).total = 0;
        particleID(i).size = 0;
        
        % Recover the ID.
        cleanID = cleanID + 1;
        availableID(cleanID) = i;
        particleID(i).ID = -3;
    end
end

end

