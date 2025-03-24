%%Erst diese Section ausführen, dann Simulink ausführen, dann fortfahren
%% 1️⃣ Roboter importieren
robotRBT = loadrobot("abbIrb120");
tInterval = [0 10];
homeConfig = robotRBT.homeConfiguration;
config = homeConfig;

TnumberOfSamples = 299;
JnumberOfSamples = 299;

%%  2️⃣ Übergabe der Simulink Parameter
%JointSpace
q_lang = squeeze(out.q)';
qd_lang = squeeze(out.qd)';
qdd_lang = squeeze(out.qdd)';
tvec = out.tvec;

step = floor(size(q_lang, 2) / JnumberOfSamples);
q = q_lang(:, 1:step:end);
qd = qd_lang(:, 1:step:end);
qdd = qdd_lang(:, 1:step:end);

%TaskSpace
jointAngles = zeros(6, TnumberOfSamples);
q_T = out.config';

%% 3️⃣ 3D Darstellung beider Bahnkurven
% Set up plot
fig = figure
show(robotRBT,homeConfig,'Frames','off','PreservePlot',false); 
set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
xlim([-1.5 1.5]), ylim([-1.5 1.5]), zlim([0 2])
hold on;

% Plot JointSpace
config = homeConfig;

for idx = 1:JnumberOfSamples
    for i = 1:6
        config(i).JointPosition = q(i,idx);

    end
    show(robotRBT,config,'Frames','off','PreservePlot',false) %Roboter anzeigen
    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    jointSpaceMarker = plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'r.','LineWidth',1);
    drawnow 
end

% Laden der Ursprungskonfiguration
show(robotRBT,homeConfig,'PreservePlot',false,'Frames','off');
config = homeConfig;
% Plot TaskSpace
for idx = 1:TnumberOfSamples

    for i = 1:6
        config(i).JointPosition = q_T(i,idx);
        jointAngles(i, idx) = config(i).JointPosition;
    end

    show(robotRBT,config,'Frames','off','PreservePlot',false)
    pointsToTravel = getTransform(robotRBT,config,"tool0","base_link"); %Endeffektor Punkte anzeigen
    taskSpaceMarker= plot3(pointsToTravel(1,4),pointsToTravel(2,4),pointsToTravel(3,4),'b.','LineWidth',1); 
    drawnow  
    pause(0.01);
end

% Legende und Titel ergänzen
legend([taskSpaceMarker jointSpaceMarker], {'Defined in Task-Space', 'Defined in Joint-Space'});
title('Manipulator Trajectories')