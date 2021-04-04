robotCurrentLocation=path(1,:);
robotGoak=path(end,:);
initialOrientation=0;
robotCurrentPose=[robotCurrentLocation initialOrientation];
robotRadius=0.3;
robot=ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotiSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);
plot(path(:,1),path(:,2),'k--d')
xlim([0 13])
ylim([0 13])
image = imread('/home/deep/amigo/map1.pgm');
bwmaze = image < 100;
gridmaze = robotics.BinaryOccupancyGrid(bwmaze);
prmSimple=robotics.PRM(gridmaze,50);
show(prmSimple)

finish = [100,50];
start = [125, 105];

path = findpath(preSimple,start,finish);

plot(ds,p);

controller = robotics.PurePursuit;
controller.Waypoints = path;
concontroller.MaxAngularVelocity = 2;
controller.DesiredLinearVelocity = 0.3;
controller.LookaheadDistance = 0.5;
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robot.getRobotPose);
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentPose = robot.getRobotPose;
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

delete(robot)
%p = [1 1 ; 2  2 ; 3 3 ; 5 5 ;6 6 ];
path = p;
rosinit;
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

for i=1:length(path)
    goalpoint.X = path(i,1)-100;
    goalpoint.Y = -(path(i,2))+100;
    pose.Position = goalpoint;
    targetpose.Pose = pose;
    goalposition.TargetPose = targetpose;
    goalid.Id = num2str(i);
    goal.Header = header;
    goal.GoalId = goalid;
    goal.Goal = goalposition;
    send(pub, goal);
%    pause(15);
    n = 1;
    
      while n == 1
         currentpoint = receive(sub,5);
         if currentpoint.Pose.Pose.Position.X == goalpoint.X
             if currentpoint.Pose.Pose.Position.Y == goalpoint.Y
                 n = 0;
             end
         end
     end
    
    
end   
rosshutdown