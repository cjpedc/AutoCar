function plot_data(odometry, laser, plotOdo, laserPlot)

% Set starting position of robot
robotPosX = odometry(1,1); robotPosY = odometry(1,2);
robot = plotRobot(robotPosX, robotPosY, 0,1);

hold on, grid on
axis equal
axis([-15 12 0 20])
h = plot(0,0,'r:','LineWidth',1);
%l = plot([0,0],[0,0],'g.','LineWidth',1);


for i = 1:length(laser)
    robotPosX = odometry(i,1);
    robotPosY = odometry(i,2);
    robotTheta = odometry(i,3);
    
    % Plot laser data
    for j = 1:size(laser,2)
        ang = j;
        if laser(i,j) < 8
            [beam,ox(i,j),oy(i,j)] = plotLaser(robotPosX,robotPosY,robotTheta,laser(i,j),ang);
            for ii = 1:size(ox,1)
                for jj = 1:size(ox,2)
                    if (ox(i,j) == ox(ii,jj)) && (oy(i,j) == oy(ii,jj))
                        exist = 1;
                    else
                        exist = 0;
                    end
                end
            end
            if (laserPlot == 1) && (exist == 0) && (j == 1 || j == 180)
                plot(ox(i,j),oy(i,j),'g.','LineWidth',1);
            end
        end
    end
    %set(beam);
    % Move and plot moved robot
    if plotOdo == 1
        delete(robot);
        robot = plotRobot(robotPosX, robotPosY, robotTheta,1);
        set(h,'XData',odometry(1:i,1),'YData',odometry(1:i,2));
    end
    drawnow
    %pause(0.001)
end

end

% OX = reshape(ox,1,size(ox,1)*size(ox,2));
% OY = reshape(oy,1,size(oy,1)*size(oy,2));
% 
% figure;hold on, grid on
% axis equal;
% axis([-15 12 0 20]);
% for i = 1:length(OX)
%     plot(OX(1,i),OY(1,i),'g.','LineWidth',1);
%     drawnow;
% end

% for i = 1:length(odometry(:,1))
%     % Move and plot moved robot
%     hold on, grid on
%     axis equal
%     axis([-12 12 0 18])
%     
%     robotPosX = odometry(i,1);
%     robotPosY = odometry(i,2);
%     robotTheta = odometry(i,3);
%     delete(robot);
%     robot = plotRobot(robotPosX, robotPosY, robotTheta,1);
%     
%     set(h,'XData',odometry(1:i,1),'YData',odometry(1:i,2));
%     drawnow
%     pause(1)
% end
