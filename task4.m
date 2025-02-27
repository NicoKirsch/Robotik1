%urdf Datei laden
robot = importrobot('abbIrb1600.urdf')

%rigidBodyTree-Objekt laden

robotRBT = loadrobot("abbIrb1600")

show(robotRBT);
homeConfig = homeConfiguration(robot);

randomConfig = randomConfiguration(robot);
show(robotRBT, randomConfig)
%task4

T_1 = getTransform(robotRBT,homeConfig,"link_1","link_2")

T_2 = getTransform(robotRBT,randomConfig,"link_1","link_2")

T_rel = inv(T_1) * T_2 %durch die relative Transformationsmatrix
%Drehung um Y

