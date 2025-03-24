%Erst diese Section ausführen, dann Simulink ausführen, dann fortfahren
%% 1️⃣ Roboter importieren & inverse Kinematik vorbereiten
robotRBT = loadrobot("abbIrb120");
tInterval = [0 10];

homeConfig = robotRBT.homeConfiguration;
config = homeConfig;
numberOfSamples = 198;

%% 2️⃣ 3D Darstellung beider Bahnkurven

show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])

hold on; % Hold the plot to add robot visualization

jointAngles = zeros(6, numberOfSamples);
q = out.config';
% Verfolgen der Bahnkurve
for idx = 1:numberOfSamples

    for i = 1:6
        config(i).JointPosition = q(i,idx);
        jointAngles(i, idx) = config(i).JointPosition;
    end

    show(robotRBT,config,'Frames','off','PreservePlot',false)
    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'b.','LineWidth',1); 
    drawnow  
    pause(0.01);
end

hold off

%% 3️⃣ Gelenkwinkel, -geschwindigkeit & -beschleunigung plotten
trajTimes = out.tTimes';

jointVelocities = diff(jointAngles, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointVelocities = [jointVelocities(:, 1), jointVelocities]; % Padding mit erster Spalte

jointAccelerations = diff(jointVelocities, 1, 2) ./ (trajTimes(2:end) - trajTimes(1:end-1));
jointAccelerations = [jointAccelerations(:, 1), jointAccelerations]; % Padding mit erster Spalte

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