%task1
%urdf Datei laden

robot = importrobot('abbIrb1600.urdf')

%show(robot);

%task2
%rigidBodyTree-Objekt laden

robotRBT = loadrobot("abbIrb1600")

show(robotRBT);


%task3
%home Position 
homeConfig = homeConfiguration(robot);
show(robotRBT, homeConfig)

randomConfig = randomConfiguration(robot);
%show(robotRBT, randomConfig)

%task4

T_1 = getTransform(robotRBT,homeConfig,"link_1","link_2")
T_1 = getTransform(robotRBT,homeConfig,"tool0","base_link")
T_2 = getTransform(robotRBT,randomConfig,"link_1","link_2")

%task 5
%inverse Kinematic Berechnung

randomConfig = randomConfiguration(robot);

ik = inverseKinematics("RigidBodyTree",robotRBT)

pos = [-0.3 0.3 1.1];
poseTF = trvec2tform(pos);

weights = [0 0 0 1 1 1];

%Berechnung der Gelenkpositionen
[configSoln,solnInfo] = ik("tool0",poseTF,weights,homeConfig);

%show(robotRBT,configSoln,PreservePlot=false);



%task6

% robot = loadrobot('abbIrb1600','DataFormat','row');
% show(robot);
% 
% analyticalIK = analyticalInverseKinematics(robot)
% showdetails(analyticalIK)
% analyticalIK.KinematicGroup
% %Der Endeffektor und die Base des Roboters sind korrekt konfiguriert
% generateIKFunction(analyticalIK,'robotIK');
% 
% %Anzeigen der gewünschten Endposition
% eePosition = [0.5 0.5 1];
% eePose = trvec2tform(eePosition);
% hold on
% plotTransforms(eePosition,tform2quat(eePose))
% hold off
% 
% ikConfig = robotIK(eePose); % Uses the generated file
% 
% figure;
% numSolutions = size(ikConfig,1)
% 
% for i = 1:size(ikConfig,1)
%     subplot(1,numSolutions,i)
%     show(robot,ikConfig(i,:));
% end


%task7

%Waypoints
wpts = [1 1 1; 0.5 1 1; 0.5 1 0.5]

[q,qd,qdd,tvec,pp] = trapveltraj(wpts,4,'PeakVelocity',1.0,'Acceleration',5)



%q = Position
%qd = Geschwindigkeit
%qdd = Beschleunigung
%tvec = wann wird welche Bewegung angefangen 


% 
% % Extrahieren der x-, y- und z-Koordinaten
% x = q(:,1);
% y = q(:,2);
% z = q(:,3);
% 
% %3D-Plot erstellen
% figure;
% plot3(x, y, z, '-o');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Positionsverlauf in 3D');
% grid on;

% figure
% subplot(4,1,1)
% plot(tvec, q)
% xlabel('t')
% ylabel('Positionen')
% legend('X','Y','Z')
% subplot(4,1,2)
% plot(tvec, qd)
% xlabel('t')
% ylabel('Geschwindigkeiten')
% legend('X','Y','Z')
% subplot(4,1,3)
% plot(tvec, qdd)
% xlabel('t')
% ylabel('Beschleunigung')
% legend('X','Y','Z')

