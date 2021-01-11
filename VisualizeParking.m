function VisualizeParking(pose, steer)
% This function creates and initalize visualizers for ego and obstacles for
% parking. This file is adapted from the "AutomatedParkingValetSimulinkExample".

% Copyright 2019 The MathWorks, Inc.

persistent vehicleBodyHandle axesHandle vehicleDims

pose(3) = rad2deg(pose(3));
steer = rad2deg(steer);

if isempty(vehicleDims)
    vehicleDims = vehicleDimensions;
end

if isempty(axesHandle) || ~isvalid(axesHandle)
    % Initialize figure
    fh1=figure('Visible','off','Position',[500, 90, 500, 500]);
    fh1.Name        = 'City Planning with RRT';
    fh1.NumberTitle = 'off';
    axesHandle      = axes(fh1);
    legend off
    axis equal;
    title(axesHandle, 'Path Planning');
    hold(axesHandle, 'on');
    
    axesHandle.XLim = [0 100];
    axesHandle.YLim = [0 100];
    
image=imread('image3.jpeg');
image=imresize(image,0.15);
grayimage=rgb2gray(image);
bwimage=grayimage < 100;
map=binaryOccupancyMap(bwimage);
show(map);
    
end

% Plot vehicle
if isempty(vehicleBodyHandle) || any(~isvalid(vehicleBodyHandle))
    vehicleBodyHandle = helperPlotVehicle(pose, vehicleDims, steer, 'Parent', axesHandle);
else
    vehicleShapes = helperVehiclePolyshape(pose, vehicleDims, steer);
    for n = 1 : numel(vehicleBodyHandle)
        vehicleBodyHandle(n).Shape = vehicleShapes(n);
    end
end
plot(axesHandle,pose(1),pose(2),'.','color','r')

fh1.Visible = 'on';
drawnow('limitrate');
end

