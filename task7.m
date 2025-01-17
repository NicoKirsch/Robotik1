robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600");
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0';
ikWeights = [0 0 0 1 1 1]; 
ikInitGuess = homeConfig; 



numberOfSamples = 50;

% Waypoints definieren

waypoints = [ 0.5  0    0.5;
              0.5  0.5  1.2;
              0.5  0    1;
              0.5  -0.5 1.2;
              0.5  0    0.5]';

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


%% Trajectory following loop
for idx = 1:numel(trajTimes)
    % Solve IK
    tgtPose = trvec2tform(q(:,idx)');
    [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;

    % Show the robot
    show(robotRBT,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))])
    
    drawnow    
end

hold off;