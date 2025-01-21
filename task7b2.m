
robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600");
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0';
ikWeights = [1 1 1 1 1 1]; 
ikInitGuess = homeConfig; 
numberOfSamples = 510;


%https://blogs.mathworks.com/student-lounge/2019/11/06/robot-manipulator-trajectory/
T0 = trvec2tform([0.5  0    0.8]);
Tf = trvec2tform([-0.5  0.8  1.2])*eul2tform([pi/2 0 pi/4],'ZYX');
tTimes = linspace(0,1,510);
tInterval = [0 5];

%Generierung der Geschwindigkeits und Beschleunigungnsverläufe
[s,sd,sdd] = trapveltraj([0 1],numel(tTimes));
[T,dT,ddT] = transformtraj(T0,Tf,tInterval,tTimes,'TimeScaling',[s;sd;sdd]);

% Set up plot
show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on; % Hold the plot to add robot visualization


plotTransforms(tform2trvec(T),tform2quat(T));


jointAngles = zeros(6, numberOfSamples);
%% Trajectory following loop
for idx = 1:numberOfSamples
    % Solve IK
    tgtPose = T(:,:,idx)
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    
    

    %Gelenkwinkel in Matrize Speichern
    for i = 1:6
        jointAngles(i, idx) = config(i).JointPosition;
    end


    % Show the robot
    show(robotRBT,config,'Frames','off','PreservePlot',false)
    
    drawnow  

end

hold off



trajTimes = tTimes;

jointVelocities = diff(jointAngles, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointVelocities = [jointVelocities(:, 1), jointVelocities]; % Padding mit erster Spalte

jointAccelerations = diff(jointVelocities, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointAccelerations = [jointAccelerations(:, 1), jointAccelerations]; % Padding mit erster Spalte

% Plot joint angles, velocities, and accelerations
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