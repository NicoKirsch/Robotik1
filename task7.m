robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600");
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0';
ikWeights = [1 1 1 1 1 1]; 
ikInitGuess = homeConfig; 



numberOfSamples = 500;

% Waypoints definieren

waypoints = [ 0.5  0    0.8;
              0.5  0.5  1.2;
              0.5  0    1;
              0.5  -0.5 1.2;
              0.5  0    0.8]';

waypointTimes = 0:4:8:12:20;

totalTime = 0.8 * (size(waypoints,1) - 1);
trajTimes = linspace(0, totalTime, numberOfSamples);


[q,qd,qdd] = trapveltraj(waypoints,numberOfSamples,'EndTime',repmat(diff(waypointTimes),[3 1]));

% Set up plot
show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on; % Hold the plot to add robot visualization


hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-')
%Darstellen der berechneten Bahnen
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));

jointAngles = zeros(6, numberOfSamples);

%% Trajectory following loop
for idx = 1:numberOfSamples
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    
    

    %Gelenkwinkel in Matrize Speichern
for i = 1:6
    jointAngles(i, idx) = config(i).JointPosition;
end


    % Show the robot
    show(robotRBT,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    
    drawnow  

end

hold off;




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