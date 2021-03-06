function [ h ] = plotRobot( xpos,ypos,theta,fignr)
figure(fignr)
X = ([-38.75, -35.00, -35.00,  -5.00,  -5.00,   5.0,   5.00,  34.0,  34.00,  38.75, 38.75, 34.00, 34.0,  5.0,  5.0, -5.0, -5.0, -35.0, -35.0, -38.75]/100);
Y = ([-27.50, -27.50, -32.50, -32.50, -27.50, -27.5, -32.50, -32.50, -27.50, -14.00, 14.00, 27.50, 32.50, 32.50, 27.5, 27.5, 32.50,  32.50,  27.5,  27.50]/100);
Xrot = xpos + (cos(theta) * X - sin(theta) * Y);
Yrot = ypos + (sin(theta) * X + cos(theta) * Y);
h = fill(Xrot,Yrot,'k');
end

