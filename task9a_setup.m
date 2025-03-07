%%
%robot = importrobot('abbIrb1600.urdf');
robotRBT = loadrobot("abbIrb1600"); % abbIrb1600/abbIrb120
homeConfig = robotRBT.homeConfiguration;
ik = inverseKinematics("RigidBodyTree",robotRBT);
eeName = 'tool0'; 
ikInitGuess = homeConfig; 
numberOfSamples = 100;%51;
%%


q_lang = squeeze(out.q)';
qd_lang = squeeze(out.qd)';
qdd_lang = squeeze(out.qdd)';
tvec = out.tvec;

step = floor(size(q_lang, 2) / numberOfSamples);
q = q_lang(:, 1:step:end);
qd = qd_lang(:, 1:step:end);
qdd = qdd_lang(:, 1:step:end);

% Set up plot
show(robotRBT,'Frames','off','PreservePlot',true); 
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on; 
config = homeConfig;

for idx = 1:numberOfSamples
    for i = 1:6
        config(i).JointPosition = q(i,idx);
        
    end
    show(robotRBT,config,'Frames','off','PreservePlot',false) %Roboter anzeigen
 
    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'r.','LineWidth',1);
    drawnow 
end
hold off;



figure;
subplot(3, 1, 1);
plot(tvec, q_lang);
xlabel('Time (s)');
ylabel('Joint Angles (rad)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 2);
plot(tvec, qd_lang);
xlabel('Time (s)');
ylabel('Joint Velocities (rad/s)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

subplot(3, 1, 3);
plot(tvec, qdd_lang);
xlabel('Time (s)');
ylabel('Joint Accelerations (rad/s^2)');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;