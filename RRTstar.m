clc
clear all
close all
%% Creating the map
% Binary Occupancy Map
image=imread('image3.jpeg');
image=imresize(image,0.15);
grayimage=rgb2gray(image);
bwimage=grayimage < 100;
mymap=binaryOccupancyMap(bwimage);
map = occupancyMatrix(mymap);
%% Setting up the vehicle
% Initialising the vehicle dimensions and inflating the map for
% collision-check.
vehicleDims = vehicleDimensions(4.98,2.189,'FrontOverhang', 0.9, 'WheelBase',2.960);
ccConfig = inflationCollisionChecker(vehicleDims,'InflationRadius',0.75);
costmap = vehicleCostmap(mymap,'CellSize',2, 'CollisionChecker', ccConfig);
% Start and goal Locations based on coordinates
startPose=[5.5, 33.5, pi/6];
goalPose= [95.5, 38.5, pi/6];
Tv=0.05; %Rate Transition for steer and pose passed to VisualizeParking
VisualizeParking(startPose,0); %Visualising the initial position/location of the car.
%% RRT* Path Planning
% Statespace and State Validator checks the map for free and
% occupied/obstacle spaces.
xlim = [0 150];   
ylim = [0 100];     
yawlim = [-pi pi]; 
bounds = [xlim;ylim;yawlim];
statespace = stateSpaceDubins(bounds);
statespace.MinTurningRadius = 2; 
statevalidator = validatorVehicleCostmap(statespace);
statevalidator.Map = costmap;
% The Path Planning for Start and Goal using the costmap
% Parameters/Properties for the planner is set for better and optimal
% results.
planner = pathPlannerRRT(costmap);
planner.MaxIterations = 20000;
Planner.ContinueAfterGoalReached=true;
planner.ApproximateSearch=false;
planner.ConnectionDistance = 4
[refPath,solnInfo] = plan(planner,startPose,goalPose);
edges = solnInfo.Edges{:,:};
nodes = solnInfo.Nodes{:,:};
%% Interpolating the Nodes - Dubin's Path
optpath = zeros(size(refPath.PathSegments,2)+1,3);
optpath(1,:) = refPath.StartPose;
for i = 1:size(refPath.PathSegments,2) 
    optpath(i+1,:) = refPath.PathSegments(i).GoalPose; 
end
optpath(:,3) = deg2rad(optpath(:,3));
path = navPath(statespace,optpath);
refstates=path.States;
%% Interpolating the Nodes - Cubic Splines
% Comment the previous section and uncomment this section to visualise for
%cubic splines as Steering function.
% [refoptpath,refDirections] = interpolate(refPath);
% approxSeparation = 0.1; % meters
% numSmoothoptpath = round(refPath.Length / approxSeparation);
% [optpath,directions] = smoothPathSpline(refoptpath,refDirections,numSmoothoptpath);
% path = navPath(statespace,optpath);
% refstates=path.States;
%% Plotting the Path after Interpolation
f = figure('Name','Driving with RRT and Pure-Pursuit');
ax = gca(f);
plot(costmap)
hold(ax, 'on');
plot(solnInfo,'EdgeColor','c','NodeColor','g','XData',nodes(:,1),'YData',nodes(:,2),'HandleVisibility','off','NodeLabel',{}) % tree expansion
plot(ax,refstates(:,1), refstates(:,2),'b-','LineWidth',2)
legend('Inflated Map(Costmap)','Reference Path')
%% Pure-Pursuit Control - Simulink
% The reference path for the controller is being set
xRef = refstates(:,1);
yRef = refstates(:,2);
tRef = refstates(:,3);
% Parameters to calculate the steering angle and a constant velocity to
% drive the vehicle
L=vehicleDims.Length;
ld=1.8;
velocity = 2;
X_o=startPose(1)
Y_o=startPose(2)
psi_o = startPose(3)
% Opening the Simulink where the simulation happens
control = 'controlpurepursuit';
open_system(control)
f = figure('Name','Driving with RRT and Pure-Pursuit');
close(f)
sim(control)

