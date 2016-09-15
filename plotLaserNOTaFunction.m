
figure; hold on, grid on
% axis equal
axis([-10 10 -10 20])
l = plot(0,0,'g.','LineWidth',2);

for i = 1:length(laser)
    for j = 1:size(laser,2)
        x(i,j) = odometry(i,1) + laser(i,j) * cos(j * (pi /180) - (pi /180));
        y(i,j) = odometry(i,2) + laser(i,j) * sin(j * (pi /180) - (pi /180));
        
        xc(i,j) = odometry(i,1);
        yc(i,j) = odometry(i,2);
        
        x(i,j) = x(i,j) - xc(i,j) ;
        y(i,j) = y(i,j) - yc(i,j) ;
        
        ox(i,j)= odometry(i,1) + (cos(odometry(i,3)-(pi/2)) * x(i,j) - sin(odometry(i,3)-(pi/2)) * y(i,j));
        oy(i,j)= odometry(i,2) + (sin(odometry(i,3)-(pi/2)) * x(i,j) + cos(odometry(i,3)-(pi/2)) * y(i,j));
        
        %beam = line([0, obsPosX], [0,obsPosY])
        beam = line([odometry(i,1), ox(i,j)], [odometry(i,2),oy(i,j)])
        set(l,'XData',ox(i,1:j),'YData',oy(i,1:j));
        
        drawnow
%         delete(beam)
        pause(0.01)
    end
end