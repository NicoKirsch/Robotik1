robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600");
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0'; 
ikInitGuess = homeConfig; 

numberOfSamples = 500;

% Waypoints definieren

%waypoints = [ 1   1  0.2   1    0.5     -1;
%              -2  0  0.2   1 -0.3    0.5]';

waypoints = [ 0   0.1796  -0.7180   0    0.5284     0;
              2.1696  0.676  -1.1939   -2.5179 1.5671    2.3713]';

%Maximale Geschwindigkeit von der langsamsten Achse gewählt in radiant
%https://library.e.abb.com/public/46ef4d53008c45fb8c6158fc92b3abdd/Datenblatt_IRB1600_lowres.pdf
[q,qd,qdd,tvec,pp] = trapveltraj(waypoints,numberOfSamples,PeakVelocity=pi);
%%
% Set up plot
%show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on; 
config = homeConfig;

for idx = 1:numberOfSamples
    for i = 1:6
        config(i).JointPosition = q(i,idx);
    end
    show(robotRBT,config,'Frames','off','PreservePlot',false) %Roboter anzeigen
 
    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'b.','LineWidth',1);
    drawnow 
end
hold off;
%%
figure;
subplot(3, 1, 1);
plot(tvec, q);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 2);
plot(tvec, qd);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 3);
plot(tvec, qdd);
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;