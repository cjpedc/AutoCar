function [] = PrintMap(name, parent, particles, overlayX, overlayY, ...
    overlayTheta)
% name - name of map file
% parent - a pointer to the specific particle you want to print out the map
% for particles - flag to indicate if position of all current particles
% should be shown on the map overlay* - if you want the specific particle's
% latest information which is just being entered into the map to show up as
% different colors, you need to specify the position of the particle here.
% Values of -1 here turn this off.

width = MAP_WIDTH;
height = MAP_HEIGHT;

for x = 1 : width
    for y = 1 : height
         map(x,y) =0;
    end
end

lastx = 0;
lasty = 0;
startx = width-1;
starty = height-1;

for x = 0 : width
    for y = 0 : height
        
        % The density of each grid square is reported, assuming a full diagonaly
        % traversal of the square. This gives good contrast.
        hit = LowComputeProb(x, y, 1.4, parent.ID); % parent->ID
        % All unknown areas are the same color. You can specify what color
        % that is later
        if (hit == UNKNOWN)
            map(x,y) = 255;
        else
            % This specifies the range of grey values for the different
            % squares in the map. Black is occupied.
            map(x,y) = uint16(230 - (hit * 230));
            % This allows us to only print out those sections of the map
            % where there is something interesting happening.
            if (x > lastx)
                lastx = x;
            end
            if (y > lasty)
                lasty = y;
            end
            if (x < startx)
                startx = x;
            end
            if (y < starty)
                starty = y;
            end
        end
    end
end

% If the command was given to print out the set of particles on the map,
% that's done here.
if (particles)
    for i = 0 : l_cur_particles_used
        if ((l_particle(i).x > 0) && (l_particle(i).x < MAP_WIDTH) && ...
                (l_particle(i).y > 0) && (l_particle(i).y < MAP_HEIGHT))
            map(uint16(l_particle(i).x), uint16(l_particle(i).y)) = 254;
        end
    end
end

% And this is where the endpoints of the current scan are visualized, if
% requested.
if (overlayX ~= -1)
    map(uint16(overlayX), uint16(overlayY)) = 254;
    for i = 0 : SENSE_NUMBER
        theta = overlayTheta + sense(i).theta;
        x = uint16(overlayX + (cos(theta) * sense(i).distance));
        y = uint16(overlayY + (sin(theta) * sense(i).distance));
        
        if ((map(x,y) < 250) || (map(x,y) == 255))
            if (sense(i).distance < MAX_SENSE_RANGE)
                if (map(x,y) < 200)
                    map(x,y) = 251;
                else
                    map(x,y) = 252;
                end
            else
                map(x,y) = 253;
            end
        end
    end
end

imshow(map,[0 255]);
imwrite(mat2gray(map),strcat(name,'.png'));

% Header file for a ppm
% sysCall = strcat(name,'.ppm');
% printFile = fopen(sysCall, 'w');
% fprintf(printFile, 'P6\n # particles.ppm \n %d %d\n', ...
%     lastx-startx+1, lasty-starty+1);
% fprintf(printFile, '255\n');

% And this is where we finally print out the map to file. Note that there
% are number of special values which could have been specified, which get
% special, non-greyscale values. Really, you can play with those colors to
% your aesthetics.
for y = lasty : -1 : starty
    for x = startx : lastx
        if (map(x,y) == 254)
            printFile(x,y) = 255;
            %             fprintf(printFile, '%c%c%c', 255, 0, 0);
            %         elseif (map(x,y) == 253)
            %             fprintf(printFile, '%c%c%c', 0, 255, 200);
            %         elseif (map(x,y) == 252)
            %             fprintf(printFile, '%c%c%c', 255, 55, 55);
            %         elseif (map(x,y) == 251)
            %             fprintf(printFile, '%c%c%c', 50, 150, 255);
            %         elseif (map(x,y) == 250)
            %             fprintf(printFile, '%c%c%c', 250, 200, 200);
            %         elseif (map(x,y) == 0)
            %             fprintf(printFile, '%c%c%c', 100, 250, 100);
        else
            printFile(x,y) = map(x,y);
            %             fprintf(printFile, '%c%c%c', map(x,y), map(x,y), map(x,y));
        end
    end
end

imshow(printFile,[0 255]);
imwrite(mat2gray(printFile),strcat(name,'.png'));

% % We're finished making the ppm file, and now convert it to png, for
% % compressed storage and easy viewing.
%   fclose(printFile);
%   img = imread(sysCall);
%   imwrite(sysCall, strcat(name,'.png'));
%   sprintf(sysCall, 'convert %s.ppm %s.png', name, name);
%   system(sysCall);
%   sprintf(sysCall, 'chmod 666 %s.ppm', name);
%   system(sysCall);
%   sprintf(sysCall, 'chmod 666 %s.png', name);
%   system(sysCall);
%   fprintf(stderr, 'Map dumped to file\n');

end

