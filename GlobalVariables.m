% Global variables

% We need to know, for various purposes, how big our map is allowed to be
global MAP_WIDTH MAP_HEIGHT;
MAP_WIDTH = 1700;
MAP_HEIGHT = 1700;

% This is the number of particles that we are keeping at the low level
global PARTICLE_NUMBER;
PARTICLE_NUMBER = 50;

% This is the number of samples that we will generate each iteration.
% Notice that we generate more samples than we will keep as actual
% particles. This is because so many are "bad" samples, and clearly won't
% be resampled, thus we don't need to allocate nearly as much memory if we
% acknowledge that a certain amount will never be considered particles.
% "Localize" function in low.m can help explain.
global SAMPLE_NUMBER;
SAMPLE_NUMBER = (PARTICLE_NUMBER * 10);

% When using hierarchical SLAM, LOW_DURATION defines how many iterations of
% the low level are performed to form one iteration of the high level. Setting this value to ridiculously high
% values can essentially turn off the hierarchical feature.
global LOW_DURATION MAP_SCALE LOW_VARIANCE MAX_SENSE_RANGE;
LOW_DURATION = 40;
MAP_SCALE = 35;
LOW_VARIANCE = 0.017 * MAP_SCALE*MAP_SCALE;
MAX_SENSE_RANGE = 7.95 * MAP_SCALE;

% Error model for motion
% Note that the var terms are really the standard deviations. See our paper on
% learning motion models for mobile robots for a full explanation of the terms.
% This model is for our ATRV Jr on a carpeted surface, at a specific period in
% in time (and thus a specific state of repair). Do not take this model as anything
% indicative of a model you should expect on your own robot.
% For a more modular design, you may find it useful to move these definitions out to
% the file "ThisRobot.c"
global meanC_D meanC_T varC_D varC_T meanD_D meanD_T varD_D varD_T ...
    meanT_D meanT_T varT_D varT_T;

meanC_D = -0.0107;
meanC_T = 0.0061;
varC_D = 0.0630;
varC_T = 2.2992;

meanD_D = 0.9577;
meanD_T = -0.1731;
varD_D = 0.1560;
varD_T = 1.9924;

meanT_D = -0.0003;
meanT_T = 0.9437;
varT_D = 0.0008;
varT_T = 0.1405;

% Threshold for culling particles.  x means that particles with prob. e^x worse
% then the best in the current round are culled
global THRESH;
THRESH = 13.0;

% Number of passes to use to cull good particles
global PASSES;
PASSES = 9;

% Maximum error we will allow for a trace in grid squares. Basically, this
% is the level of of "background noise", or the base chance that something 
% weird and completely random happened with the sensor, not conforming to 
% our observation model.
global MAX_TRACE_ERROR;
MAX_TRACE_ERROR = exp(-24.0/LOW_VARIANCE);
% A constant used for culling in Localize
global WORST_POSSIBLE;
WORST_POSSIBLE = -10000;

% Used for recognizing the format of some data logs.
global LOG REC;
LOG = 0;
REC = 1;

% The number of sensor readings for this robot (typically 181 for a laser 
% range finder)
global SENSE_NUMBER;
SENSE_NUMBER = 180;
% Turn radius of the robot in map squares. Since the "robot" is actually the 
% sensor origin for the purposes of this program, the turn radius is the 
% displacement of the sensor from the robot's center of rotation (assuming 
% holonomic turns)
global TURN_RADIUS;
TURN_RADIUS = (0.40 * MAP_SCALE);

global THold Tsense;
Tsense = struct('theta',0,'distance',0);
THold = struct('C',0,'D',0,'T',0,'sense',Tsense);

global TOdo odometry;
TOdo = struct('x',0,'y',0,'theta',0);
odometry = TOdo;