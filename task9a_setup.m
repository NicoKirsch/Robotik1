%Erst diese Section ausführen, dann Simulink ausführen, dann fortfahren
%% 1️⃣ Roboter importieren
robotRBT = loadrobot("abbIrb120");
homeConfig = robotRBT.homeConfiguration;

numberOfSamples = 200;

%%  2️⃣ Übergabe der Simulink Parameter

q_lang = squeeze(out.q)';
qd_lang = squeeze(out.qd)';
qdd_lang = squeeze(out.qdd)';
tvec = out.tvec;

step = floor(size(q_lang, 2) / numberOfSamples);
q = q_lang(:, 1:step:end);
qd = qd_lang(:, 1:step:end);
qdd = qdd_lang(:, 1:step:end);

%% 3️⃣ 3D Darstellung beider Bahnkurven
show(robotRBT,'Frames','off','PreservePlot',false); 
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


%%
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


%%
config = homeConfig;
config(1).JointPosition =1;
config(2).JointPosition =1;
config(3).JointPosition =0.2;
config(4).JointPosition =1;
config(5).JointPosition =0.5;
config(6).JointPosition =-1;

config1 = homeConfig;
config1(1).JointPosition =-2;
config1(2).JointPosition =0;
config1(3).JointPosition =0.2;
config1(4).JointPosition =1;
config1(5).JointPosition =-0.3;
config1(6).JointPosition =0.5;

start = getTransform(robotRBT,config,"tool0","base_link")
ende = getTransform(robotRBT,config1,"tool0","base_link")