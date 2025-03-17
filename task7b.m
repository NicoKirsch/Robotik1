%Die Sectionen müssen einzeln ausgeführt werden

%% 1️⃣ Roboter importieren & inverse Kinematik vorbereiten
robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600");
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0';
ikWeights = [1 1 1 1 1 1]; 
ikInitGuess = homeConfig; 
numberOfSamples = 51;

%% 2️⃣ Waypoints & Zeiten definieren (task7b, sonnst nicht nutzen)
T0 = trvec2tform([0.8150  0    1.2615])*eul2tform([pi pi/2 pi],'ZYX');
Tf = trvec2tform([-0.5  0.8  1.2])*eul2tform([pi/2 0 pi/4],'ZYX');

tTimes = linspace(0,1,51);
tInterval = [0 5];

%%  3️⃣  Waypoints & Zeiten definieren (task7c, sonnst nicht nutzen)
T0 = trvec2tform([0.8150  0    1.2615])*eul2tform([pi pi/2 pi],'ZYX');
Tf = trvec2tform([0.8150 0 0.6615])*eul2tform([pi pi/2 pi],'ZYX');

%% 4️⃣ Generierung der Geschwindigkeits und Beschleunigungnsverläufe
[s,sd,sdd] = trapveltraj([0 1],numel(tTimes));
[T,dT,ddT] = transformtraj(T0,Tf,tInterval,tTimes,'TimeScaling',[s;sd;sdd]);

%% 5️⃣ 3D Bahnkurve
show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on; % Hold the plot to add robot visualization

plotTransforms(tform2trvec(T),tform2quat(T));

jointAngles = zeros(6, numberOfSamples);

for idx = 1:numberOfSamples
    % Lösen IK
    tgtPose = T(:,:,idx)
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    
    %Gelenkwinkel in Matrize Speichern
    for i = 1:6
        jointAngles(i, idx) = config(i).JointPosition;
    end
    show(robotRBT,config,'Frames','off','PreservePlot',false)  
    drawnow  
end

hold off

trajTimes = tTimes;
%% 6️⃣ Gelenkgeschwindigkeiten & -beschleunigungen berechnen
jointVelocities = diff(jointAngles, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointVelocities = [jointVelocities(:, 1), jointVelocities]; % Padding mit erster Spalte

jointAccelerations = diff(jointVelocities, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointAccelerations = [jointAccelerations(:, 1), jointAccelerations]; % Padding mit erster Spalte

%% 7️⃣ Gelenkwinkel, -geschwindigkeit & -beschleunigung plotten
% Ploten der Gelenkwinkel, -geschwindigkeiten und -beschleunigungen
figure;
subplot(3, 1, 1);
plot(trajTimes, jointAngles');
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 2);
plot(trajTimes, jointVelocities');
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 3);
plot(trajTimes, jointAccelerations');
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;