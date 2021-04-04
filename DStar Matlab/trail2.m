image = imread('/home/deep/amigo/conference_local2.pgm');
map = image < 100;
finish = [108,108];
start = [61,80];

%% Binary Occupoancy grid
occupancy = robotics.BinaryOccupancyGrid(map);
occu = occupancyMatrix(occupancy);
pathplanning = Dstar(occu);
pathplanning.plan(finish);
p = pathplanning.query(start);

plot(pathplanning,p);

%% Rostopics publishing
path = p;
rosinit;
[publish, g] = rospublisher('move_base/goal','move_base_msgs/MoveBaseActionGoal');
header = rosmessage('std_msgs/Header');
goalid = rosmessage('actionlib_msgs/GoalID');
goalposition = rosmessage('move_base_msgs/MoveBaseGoal');
target = rosmessage('geometry_msgs/PoseStamped');
sub = rossubscriber('pose');
pose = rosmessage('geometry_msgs/Pose');
quaternion = rosmessage('geometry_msgs/Quaternion');
goal = rosmessage('geometry_msgs/Point');

quaternion.W = 1;
header.FrameId = 'base_link';
pose.Orientation = quaternion;
g.Header = header;
header.FrameId = 'odom';
target.Header = header;

%% Reach Goal

for i=1:length(path)
    goal.X = path(i,1)-start(1);
    goal.Y = -(path(i,2))+start(2);
    pose.Position = goal;
    target.Pose = pose;
    goalposition.TargetPose = target;
    goalid.Id = num2str(i);
    g.Header = header;
    g.GoalId = goalid;
    g.Goal = goalposition;
    send(publish, g);
    n = 1;
    
      while n == 1
         currentpoint = receive(sub,5);
         if currentpoint.Pose.Pose.Position.X == goal.X
             if currentpoint.Pose.Pose.Position.Y == goal.Y
                 n = 0;
                 pickNplace;
             end
         
         end
      end
    pickNplace;
end 

rosshutdown