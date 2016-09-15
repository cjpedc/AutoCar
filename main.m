
logFile = 'loop5.log';

[odometry, laser] = ReadLog(logFile);

plotOdo = 1;
laserPlot = 1;
plot_data(odometry, laser, plotOdo, laserPlot);