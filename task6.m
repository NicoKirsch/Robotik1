%% 1️⃣ task6
robot = loadrobot('abbIrb1600','DataFormat','row');
analyticalIK = analyticalInverseKinematics(robot)
showdetails(analyticalIK)
analyticalIK.KinematicGroup

%Der Endeffektor und die Base des Roboters sind korrekt konfiguriert
generateIKFunction(analyticalIK,'robotIK');

%Positionen definieren
eePosition = [-0.3 0.3 1.1];    %normale Position
eePosition1 = [0.15 0 1.58];     %nahe einer Singularität
eePosition2 = [0.8150 0 0.9615];   %Singularität

%Transformationsmatrix berechnen
eePose = trvec2tform(eePosition);
eePose1 = trvec2tform(eePosition1);
eePose2 = trvec2tform(eePosition2)*eul2tform([pi pi/2 pi],'ZYX');

%Zeiten messen
tic
    ikConfig = robotIK(eePose);
toc
tic
    ikConfig1 = robotIK(eePose1);
toc
tic
    ikConfig2 = robotIK(eePose2);
toc

%Zeiten ausgeben
fprintf('number of solutions: %f\n', size(ikConfig,1))
fprintf('number of solutions: %f\n', size(ikConfig1,1))
fprintf('number of solutions: %f\n', size(ikConfig2,1))

%% 3D normale Position
numSolutions = size(ikConfig, 1);
for i = 1:numSolutions
    if i <= numSolutions / 2
        subplot(2, numSolutions / 2, i);
        show(robot, ikConfig(i, :));
    else
        id = i - numSolutions / 2;
        subplot(2, numSolutions / 2, i); 
        show(robot, ikConfig(i, :));
    end
end

%% 3D nahe einer Singularität
numSolutions = size(ikConfig1, 1);
for i = 1:numSolutions
    if i <= numSolutions / 2
        subplot(2, numSolutions / 2, i);
        show(robot, ikConfig1(i, :));
    else
        id = i - numSolutions / 2;
        subplot(2, numSolutions / 2, i); 
        show(robot, ikConfig1(i, :));
    end
end

%% 3D in einer Singularität
numSolutions = size(ikConfig2, 1);
numRows = 2;
numCols = ceil(numSolutions / numRows);

for i = 1:numSolutions
    subplot(numRows, numCols, i);
    show(robot, ikConfig2(i, :));
end