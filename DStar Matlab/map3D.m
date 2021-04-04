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

%% Reach Goal

for i=1:length(path)
    goalpoint.X = path(i,1);
    goalpoint.Y = path(i,2);
    pose.Position = goalpoint;
    targetpose.Pose = pose;
    goalposition.TargetPose = targetpose;
    goalid.Id = num2str(i);
    goal.Header = header;
    goal.GoalId = goalid;
    goal.Goal = goalposition;
    send(pub, goal);
    n = 1;
    
      while n == 1
         currentpoint = receive(sub,5);
         if currentpoint.Pose.Pose.Position.X == goalpoint.X
             if currentpoint.Pose.Pose.Position.Y == goalpoint.Y
                 n = 0;
             end
       
         end 
             
    
    
end   
rosshutdown