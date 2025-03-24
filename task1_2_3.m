%% 1️⃣ task1
%urdf Datei laden
robot = importrobot('abbIrb1600.urdf')
robot = importrobot('abbIrb120.urdf')
%show(robot);

%% 2️⃣ task2
%rigidBodyTree-Objekt laden

robotRBT = loadrobot("abbIrb1600")
%show(robotRBT);

%% 3️⃣ task3
%home Position 
homeConfig = homeConfiguration(robot);
show(robotRBT, homeConfig)

randomConfig = randomConfiguration(robot);
%show(robotRBT, randomConfig)

%task4

T_1 = getTransform(robotRBT,homeConfig,"link_1","link_2")
T_1 = getTransform(robotRBT,homeConfig,"tool0","base_link")
T_2 = getTransform(robotRBT,randomConfig,"link_1","link_2")

%%
randomConfig = randomConfiguration(robot);

ik = inverseKinematics("RigidBodyTree",robotRBT)

pos = [-0.3 0.3 1.1];
poseTF = trvec2tform(pos);

weights = [0 0 0 1 1 1];

%Berechnung der Gelenkpositionen
[configSoln,solnInfo] = ik("tool0",poseTF,weights,homeConfig);

show(robotRBT,configSoln,PreservePlot=false);


