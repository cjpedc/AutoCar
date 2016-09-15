function [beam, ox, oy ] = plotLaser(robotPosX,robotPosY,robotTheta,laser,ang)

x = robotPosX + laser * cos(ang * (pi /180) - (pi /180));
y = robotPosY + laser * sin(ang * (pi /180) - (pi /180));

xc = robotPosX;
yc = robotPosY;

x = x - xc;
y = y - yc;

ox = robotPosX + (cos(robotTheta-(pi/2)) * x - sin(robotTheta-(pi/2)) * y);
oy = robotPosY + (sin(robotTheta-(pi/2)) * x + cos(robotTheta-(pi/2)) * y);

%beam = line([robotPosX, ox], [robotPosY,oy]);
beam = 0;
end

