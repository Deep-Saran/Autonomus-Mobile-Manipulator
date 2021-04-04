function pickNplace

%Defining robot parameters

% syms theta1 theta2 theta3 theta4 theta5 d1 a3 a4 d5

% d1=13.57;

% a3=11.52;

% a4=6.21;

% d5=6.39;

d1=4.5;

a3=13.5;

a4=12.5;

d5=4.5;

% theta1=0;

% theta2=0;

% theta3=0;

% theta4=0;

% theta5=0;

pi=3.14;

% dh1=Revolute('d',d1,'a',0,'alpha',0);

% dh2=Revolute('d',0,'a',0,'alpha',90);

% dh3=[theta3     0 a3   0];

% dh4=[theta4-1.57  0 a4   0];

% dh5=[theta5    d5  0   -1.57];

% R=SerialLink(dh1*dh2*dh3*dh4*dh5)



dh=[ 0 d1 0 0

    0  0  10 1.57

    0  0 a3  0

    -1.57 0 a4 0

    0   d5 0 -1.57];

R=SerialLink(dh);

% R.teach();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Same robot can also be defined as-

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L(1) = Link([0 d1 0 pi/2]);

L(2) = Link([0 0 10 0]);

L(3) = Link([0 0 a3 0]);

L(4) = Link([-1.57 0 a4 0]);

L(5) = Link([0 d5 0 -1.57]);

% Joint limits

L(1).qlim = pi/180*[-90 90];

L(2).qlim = pi/180*[-90 90];

L(3).qlim = pi/180*[-90 90];

L(4).qlim = pi/180*[-90 90];

L(5).qlim = pi/180*[-90 90];

% create the robot model

Robot = SerialLink(L);

Robot.name = "Robot";

%Understanding the performance of the robot

% Robot.teach();

Robot.fkine([0 0 0 0 0])

%          1         0         0        36

%          0         1         0      -4.5

%          0         0         1     4.504

%          0         0         0         1

Robot.fkine([0 90 0 0 0])

%    -0.4481   -0.0007   -0.8940    -16.13

%     0.0007    1.0000   -0.0012    -4.474

%     0.8940   -0.0012   -0.4481     36.69

%          0         0         0         1 

Robot.fkine([0 90 -90 0 0])

%          1         0         0     21.52

%          0         1         0    -4.493

%          0         0         1     13.44

%          0         0         0         1

figure

Robot.plot([0 90 -90 0 0]);

pause(5)

% figure

% Robot.plot([0 90 0 0 0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a = arduino('COM4', 'Uno', 'Libraries', 'Servo');

% s = servo(a, 'D4', 'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2500*10^-6)

% %  MinPulseDuration: 5.00e-04 (seconds)

% %  MaxPulseDuration: 2.50e-03 (seconds)

%  current_pos = readPosition(s);

%  writePosition(s, angle);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create the new ArmRobot object

%When the value is sent by the xtion/search algorithm, set goal=1

goal=1;

while (goal)

    %Goal Position

    goalx=30;

    goaly=-4.023;

    goalz=-20.484; 

    

robot = ArmRobot('/dev/ttyUSB0');

% values used Throughout

joints = [0 1 2 3 4 5];

% %testing base

% testbasec=[1500 1500 1500 1500 1500 1500];

% testbasel=[650 1500 1500 1500 1500 1500];

% testbaser=[2300 1500 1500 1500 1500 1500];

% %testing shoulder

% testshoulderup=[1500 1900 1500 1500 1500 1500];%moves up

% testshoulderc=[1500 1500 1500 1500 1500 1500];%shoulder home

% testshoulderc=[1500 1200 1500 1500 1500 1500];%down

% %testing elbow

% testelbowc=[1500 1500 2000 1500 1500 1500];%center

% %testing wrist

% testwristc=[1500 1500 2000 1600 1500 1500];%wrist center

% testwristd=[1500 1500 2000 1000 1500 1500]%wrist down

% testwristu=[1500 1500 2000 1800 1500 1500]%wrist up

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% testwristtwc1=[1500 1500 2000 1400 2000 1500];%wrist twist max

% testwristtwc=[1500 1500 2000 1400 800 2000];%wrist twist min

% %testing grip

% testgrip=[1500 1600 1500 1500 1500 1200];

% opengrip=[1500 1600 1500 1500 1500 900];

% Set configuration of the robot

robot.setServoCenters([1450 1500 1400 1475 1480 1575]);

robot.setServoBounds([555 800 900 575 575 900],[2350 2150 2000 2400 2350 2250]);

robot.setLinkLengths([3.66 4.2 1.69 13.57 12.52 6.21 4.78]);

% Connect to the Robot

robot.connect();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% q=[1,0,0,0,0,0];

% q = R.ikine_sym(6)

% pause(5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%POSITION / INVERSE KINEMATICS/HOME

T=[1 0 0  21.52

    0 1 0 -4.493

    0 0 1 13.44

    0 0 0 1];

%q=[13 15 12];

qreach=Robot.ikcon(T)

% HOME

% figure

Robot.plot(qreach);%Reaches home through different configuration



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Taking avg degree to servo conversion and using

 c=[1500 1500 1500 1500 1475 1000];%C

% qreachservo=qreach*9.5*180/3.14;

% qreachservo(6)=0

% qreachservo1=zeros(1,6);

% for counter=1:1:6

%     if counter==2

%     qreachservo1(counter)=c(counter)+qreachservo(counter)-900;

%     elseif counter==1

%             qreachservo1(counter)=c(counter)+qreachservo(counter);

%             elseif counter ==3

%             qreachservo1(counter)=c(counter)+qreachservo(counter)+900;    

%     elseif counter ==4

%         qreachservo1(counter)=c(counter)-qreachservo(counter);

%    

%     else

%         qreachservo1(counter)=c(counter)-qreachservo(counter);

%     end

% end

% display(qreachservo1);

% %robot.moveJoints(c,joints);

% pause(0.5);

% robot.moveJoints(qreachservo1,joints);

% R.teach();

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%qreach1=qreachservo1;

%qreach1(6)=1800;

%robot.moveJoints(qreach1,joints);

%All that was setting home position 

% %Relative motion from current position to goal position.

%Assuming we get the x y z value

Tgoal=[1 0 0  goalx

    0 1 0 goaly

    0 0 1 goalz

    0 0 0 1];

qreachgoal=R.ikcon(Tgoal)

% R.plot(qreachgoal);

qreachservogoal=qreachgoal*10*180/3.14;

qreachservogoal(6)=0;

qreachservogoal1=zeros(1,6);

for counterg=1:1:6

    if counterg==2

    qreachservogoal1(counterg)=c(counterg)+qreachservogoal(counterg)-400;

    elseif counterg==1

            qreachservogoal1(counterg)=c(counterg)+qreachservogoal(counterg);

            elseif counterg ==3

            qreachservogoal1(counterg)=c(counterg)+qreachservogoal(counterg)+500;    

    elseif counterg ==4

        qreachservogoal1(counterg)=c(counterg)-qreachservogoal(counterg);

   

    else

        qreachservogoal1(counterg)=c(counterg)-qreachservogoal(counterg)+200;

    end

end

display(qreachservogoal1);

%robot.moveJoints(c,joints);

%//pause(5);

robot.moveJoints(qreachservogoal1,joints);

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %Relative motion from current position to goal position.

qreach1=qreachservogoal1

qreach1(6)=900;

pause(1);

robot.moveJoints(qreach1,joints);

pause(2);

qreachpick=qreach1;
qreachpick(6)=1800;
robot.moveJoints(qreachpick,joints);

%pick up and move back

%c1=[1500 1500 1000 1500 1475 1000];%C1

qup=qreachpick;

qup(3)=qreach1(3)-400;

robot.moveJoints(qup,joints);

pause(1);

qreachservogoal2=qup;

qreachservogoal2(1)=qup(1)-500;

robot.moveJoints(qreachservogoal2,joints);

pause(3);

qreachservogoal3=qreachservogoal2;
qreachservogoal3(3)=qreachservogoal2(3)+400;

qreachservogoal3(6)=1000;

robot.moveJoints(qreachservogoal3,joints);

pause(3);

robot.moveJoints(qup,joints);



%%setting the simulation also back to home

% robot.moveJoints(c,joints);

Th=[1 0 0  26

    0 1 0 -4.023

    0 0 1 -10.484

    0 0 0 1];

qh=[13 15 12];

qreachh=R.ikcon(Th)

% R.teach();

% q = R.jtraj(Tgoal,Th)

goal=0

end

% pause(5);

% % robot.moveJoints(qreachservo,joints);

% robot.moveJoints(testbasec,joints);

% robot.moveRelative(qreach1,joints);

% qreach2=[qreach 10];

% robot.moveRelative(qreach2,joints);

% pause(0.5);

% c1=[1500 1500 1800 1400 800 1800];%C1

% robot.moveJoints(c1,joints);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% g = PGraph();

% g.add_node([1 2]');  % add node 1

% g.add_node([4 4]');  % add node 1

% g.add_node([1 3]');  % add node 1

% g.add_edge(1, 2);    % add edge 1-2

% g.add_edge(2, 3);    % add edge 2-3

% g.add_edge(1, 3);    % add edge 1-3

% g.plot()

% map = robotics.BinaryOccupancyGrid(20,20,20);

% xy = [5 5; 4.3 4.4];% 5.6 5.3];

% setOccupancy(map,xy,1);

% map=readOccupancyMap3D(map.ot);

% width=5;

% height=5;

% map = robotics.BinaryOccupancyGrid(width,height);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Path Planning 

% load map          % load map

% goal = [5,3,2];

% start=[3,1,1];

% ds = Dstar(map);    % create navigation object

% ds.plan(goal)       % create plan for specified goal

% ds.path(start)      % animate path from this start location

% ds.plot(goal); 

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %  

% function moveLinear(presentpos,goalpos,joints,robot)

%              temppos=presentpos;

%              for i=1:1:6

%                  if presentpos(i)<goalpos(i)

%                  while(presentpos(i)<goalpos(i))

%                      tempose(i)=temppos(i)+1;

%                      robot.moveJoints(temppos,joints);

%                  end

%                  elseif presentpos(i)>goalpos(i)

%                  while( presentpos(i)>goalpos(i))

%                      tempose(i)=temppos(i)-1;

%                      robot.moveJoints(temppos,joints);

%                  end

%                  end

%              end

% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% R.traj(qreach,qreachgoal);







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  function linearIncrement(goalpos,presentposition,joints)

%  presentpos=presentposition;

%   for i=6;-1;1

%     if goalpos(i)<presentpos(i)

%       while goalpos(i)>presentpos(i)

%       presentpos(i)= presentpos(i)+10;

%       robot.moveJoints(presentpos,joints);  

%       end

%     else

%     while(goalpos(i)<presentpos(i))

%       presentpos(i)=presentpos(i)-10;

%       robot.moveJoints(presentpos,joints);

%     end

%     end

%   end

%  end

end