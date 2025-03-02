%% 1️⃣ Roboter importieren & inverse Kinematik vorbereiten
robot = loadrobot("abbIrb1600");  
homeConfig = robot.homeConfiguration;  
ik = inverseKinematics("RigidBodyTree", robot);  
eeName = 'tool0';  
ikWeights = [1 1 1 1 1 1];  
ikInitGuess = homeConfig;  

%% 2️⃣ Waypoints & Zeiten definieren (Kartesische Koordinaten)
waypoints = [ 0.5  0    0.8;
              0.5  0.5  1.2;
              0.5  0    1;
              0.5  -0.5 1.2;
              0.5  0    0.8]';
waypointTimes = 0:4:8:12:20;  

numberOfSamples = 500;  
trajTimes = linspace(0, waypointTimes(end), numberOfSamples);  

%% 3️⃣ Inverse Kinematik für die Wegpunkte berechnen (Kartesisch → Gelenkwinkel)
numJoints = length(homeConfig);  
jointWaypoints = zeros(numJoints, size(waypoints, 2));  

for i = 1:size(waypoints, 2)  
    tgtPose = trvec2tform(waypoints(:, i)');  % Kartesische Pose → Homogene Transformationsmatrix  
    [config, info] = ik(eeName, tgtPose, ikWeights, ikInitGuess);  
    ikInitGuess = config; % Nächste IK-Lösung startet hier  

    % Gelenkwinkel speichern
    for j = 1:numJoints  
        jointWaypoints(j, i) = config(j).JointPosition;  
    end  
end  

%% 4️⃣ Gelenkwinkel interpolieren mit trapveltraj
[q, qd, qdd] = trapveltraj(jointWaypoints, numberOfSamples, ...
                          'EndTime', repmat(diff(waypointTimes), [numJoints, 1]));
% Um Bahnpunkteeinzuzeichnen
[q2, qd2, qdd2] = trapveltraj(waypoints, numberOfSamples, ...
                          'EndTime', repmat(diff(waypointTimes), [3, 1]));

%% 5️⃣ Roboter-Trajektorie animieren
show(robot, homeConfig, 'Frames', 'off', 'PreservePlot', true);
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2]);  
hold on;


hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-')
%Darstellen der berechneten Bahnen
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
set(hTraj,'xdata',q2(1,:),'ydata',q2(2,:),'zdata',q2(3,:));

jointAngles = q;  

for idx = 1:numberOfSamples  
    config = homeConfig;  
    for j = 1:numJoints  
        config(j).JointPosition = jointAngles(j, idx);  
    end  
    
    show(robot, config, 'Frames', 'off', 'PreservePlot', false);  

    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'r.','LineWidth',1);
    
    title(['Trajectory at t = ' num2str(trajTimes(idx))]);  
    drawnow;  
end  

hold off;  

%% 6️⃣ Gelenkgeschwindigkeiten & -beschleunigungen berechnen
jointVelocities = diff(jointAngles, 1, 2) ./ diff(trajTimes);
jointVelocities = [jointVelocities(:,1), jointVelocities];  % Erste Spalte kopieren

jointAccelerations = diff(jointVelocities, 1, 2) ./ diff(trajTimes);
jointAccelerations = [jointAccelerations(:,1), jointAccelerations];  % Erste Spalte kopieren

%% 7️⃣ Gelenkwinkel, -geschwindigkeit & -beschleunigung plotten
figure;
subplot(3,1,1);
plot(trajTimes, jointAngles');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3,1,2);
plot(trajTimes, jointVelocities');
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3,1,3);
plot(trajTimes, jointAccelerations');
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;
