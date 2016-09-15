function [odometry, laser] = ReadLog(logFile)
% Reads back into the sensor data structures the raw readings that were 
% stored to file. Reads a single line from the file, and interprets it by 
% its delineator (either Laser or Odometry).


FID = fopen(logFile, 'r');
if FID == -1, error('Cannot open file'), end
Data = textscan(FID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(FID);


or = 0; lr = 0;
for i = 1:length(CStr)
    s = Data{1}{i};
    s = strsplit(s);
    o = 1; l = 1;
    if strcmp(s(1),'Odometry')
        or = or + 1;
        for j = 2:length(s)-1
            odo(or,o) = str2double(s(j));
            o = o + 1;
        end
    elseif strcmp(s(1),'Laser')
        lr = lr + 1;
        for j = 3:length(s)-1
            laser(lr,l) = str2double(s(j));
            l = l + 1;
        end
    end
end

odometry(:,1) = odo(:,1);
odometry(:,2) = odo(:,2);
odometry(:,3) = odo(:,3);

for i = 1:length(odometry(:,3))
    if(odometry(i,3) > pi)
        odometry(i,3) = odometry(i,3) - (2 * pi);
    elseif(odometry(i,3) < -pi)
        odometry(i,3) = odometry(i,3) + (2 * pi);
    end
end

end

% odometry(:,1) = odo(:,1) / 100.0;
% odometry(:,2) = odo(:,2) / 100.0;
% odometry(:,3) = odo(:,3) * pi / 180.0;

% MAP_SCALE = 35;
% TURN_RADIUS = 0.40 * MAP_SCALE;
% MAX_SENSE_RANGE = 7.95 * MAP_SCALE;

% odometry(:,1) = odometry(:,1) - (cos(odometry(:,3)) * TURN_RADIUS / MAP_SCALE);
% odometry(:,2) = odometry(:,2) - (sin(odometry(:,3)) * TURN_RADIUS / MAP_SCALE);

% laser = laser * MAP_SCALE / 100.0;
% laser = laser;

% temp = reshape(laser, 1, size(laser,1)*size(laser,2));
% for i = 1:length(temp)
%     if temp(i) > MAX_SENSE_RANGE
%         temp(i) = MAX_SENSE_RANGE;
%     end
% end
% laser = reshape(temp, size(laser,1), size(laser,2));
