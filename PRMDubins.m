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
costmap = vehicleCostmap(mymap,'CellSize',1.5, 'CollisionChecker', ccConfig);
inflate(mymap,0.3)
startPose= [5.5, 33.5, pi/6];
goalPose=[95.5, 38.5, pi/2];
Tv=0.05; %Rate Transition for steer and pose 
VisualizeParking(startPose,0);
%% PRM Path Planning
% Statespace and State Validator checks the map for free and
% occupied/obstacle spaces.
xlim = [0 100];   
ylim = [0 100];     
yawlim = [-3.1416 3.1416]; 
bounds = [xlim;ylim;yawlim];
statespace = stateSpaceDubins(bounds);
statevalidator = validatorVehicleCostmap(statespace);
statevalidator.Map = costmap;
%% prm-astar-dubins
% Initialise Start and Goal for the planner
start= [5.5, 33.5];
goal=[95.5, 38.5];
startorientation=0;
goalorientation=0;
% PRM and Graph search
myprm = mobileRobotPRM(mymap,500);
optpath=findpath(myprm,start,goal);
optpath(1,3)=startorientation;
optpath(size(optpath,1),3)=goalorientation;
% Define orientation for middle points as (a)=atan(dy/dx) from one point to
% another
for i=size(optpath,1)-1:-1:2
    dy=optpath(i+1,2)-optpath(i,2);
    dx=optpath(i+1,1)-optpath(i,1);
    optpath(i,3)=atan((dy)/(dx))
    if dx<0 & dy>0 
        optpath(i,3)=pi-abs(optpath(i,3))
    elseif dx<0 & dy<0 
    optpath(i,3)=pi+abs(optpath(i,3))
    elseif dx>0 & dy<0 
         optpath(i,3)=2*pi-abs(optpath(i,3))
    end
end
% Connectwith dubins path for each vertex with the next
dubConnObj = dubinsConnection;
dubConnObj.MinTurningRadius = 2;
for i=1:size(optpath,1)-1
[pathSegObj(i), pathCosts] = connect(dubConnObj,optpath(i,:),optpath(i+1,:));
end
%% Interpolating the Nodes - Dubin's Path
interpath = zeros(size(pathSegObj,2)+1,3);
interpath(1,:) = pathSegObj{1,1}.StartPose;
for i = 1:size(pathSegObj,2) 
    interpath(i+1,:) = pathSegObj{1,i}.GoalPose; 
end
interpath(:,3) = deg2rad(interpath(:,3));
path = navPath(statespace,interpath);
refstates=path.States
%% Plotting the Path for PRM
path = navPath(statespace,interpath);
f = figure('Name','Driving with PRM and Pure-Pursuit');
ax = gca(f);
plot(costmap)
hold(ax, 'on');
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
sim(control)