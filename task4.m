%% 1️⃣ task4
robot = importrobot('abbIrb1600.urdf')
robotRBT = loadrobot("abbIrb1600")

show(robotRBT);
homeConfig = homeConfiguration(robot);

randomConfig = randomConfiguration(robot);
show(robotRBT, randomConfig)

T_1 = getTransform(robotRBT,homeConfig,"link_1","link_2")

T_2 = getTransform(robotRBT,randomConfig,"link_1","link_2")

%Berechnung der relativen Transformationsmatrix
T_rel = inv(T_1) * T_2 

%3D beide Konfigurationen darstellen
subplot(1,2,1)
show(robotRBT,homeConfig(1,:));
subplot(1,2,2)
show(robotRBT,randomConfig(1,:));
