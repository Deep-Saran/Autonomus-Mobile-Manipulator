image = imread('/home/deep/amigo/conference_local2.pgm');
bwmaze = image < 100;
%% Binary Occupoancy grid
gridmaze = robotics.BinaryOccupancyGrid(bwmaze);
occmaze = occupancyMatrix(gridmaze);
ds = Dstar(occmaze);
finish = [92,92];
start = [60,70];
ds.plan(finish);
p = ds.query(start);

plot(ds,p);

%% Rostopics publishing
%p = [1 1 ; 2  2 ; 3 3 ; 5 5 ;6 6 ];
path = p;

[pub, goal] = rospublisher('move_base/goal','move_base_msgs/MoveBaseActionGoal');
header = rosmessage('std_msgs/Header');
goalid = rosmessage('actionlib_msgs/GoalID');
goalposition = rosmessage('move_base_msgs/MoveBaseGoal');
targetpose = rosmessage('geometry_msgs/PoseStamped');
sub = rossubscriber('pose');
pose = rosmessage('geometry_msgs/Pose');
quaternion = rosmessage('geometry_msgs/Quaternion');
goalpoint = rosmessage('geometry_msgs/Point');
quaternion.W = 1;
header.FrameId = 'base_link';
pose.Orientation = quaternion;
goal.Header = header;
header.FrameId = 'map';
targetpose.Header = header;
tftree = rostf;
plot(path(:,1),path(:,2),'k--d');

controller = robotics.PurePursuit('Waypoints',path);
controller.DesiredLinearVelocity = 0.4;
controlRate = robotics.Rate(10);
goalRadius = 0.1;
robotCurrentLocation = path(1,:);
updateCounter = 1;
while( distanceToGoal > goalRadius )
    % Receive a new laser sensor reading.
    scanMsg = receive(sub);
    
    % Get robot pose at the time of sensor reading.
    pose = getTransform(tftree, 'map', 'robot_base', scanMsg.Header.Stamp, 'Timeout', 2);
    
    % Convert robot pose to 1x3 vector [x y yaw].
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    
    % Extract the laser scan.
    scan = lidarScan(scanMsg);
    ranges = scan.Ranges;
    ranges(isnan(ranges)) = sim.LaserSensor.MaxRange;
    modScan = lidarScan(ranges, scan.Angles);
    
    % Insert the laser range observation in the map.
    insertRay(map, robotPose, modScan, sim.LaserSensor.MaxRange);
    
    % Compute the linear and angular velocity of the robot and publish it
    % to drive the robot.
    [v, w] = controller(robotPose);
    velMsg.Linear.X = v;
    velMsg.Angular.Z = w;
    send(velPub, velMsg);
    
    % Visualize the map after every 50th update.
    %     if ~mod(updateCounter,50)
    %         mapHandle.CData = occupancyMatrix(map);
    %         title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    %     end
    
    % Update the counter and distance to goal.
    updateCounter = updateCounter+1;
    distanceToGoal = norm(robotPose(1:2) - robotGoal);
    
    % Wait for control rate to ensure 10 Hz rate.
    waitfor(controlRate);
end
robotGoal = path(end,:);
distanceToGoal = norm(robotCurrentLocation - robotGoal);