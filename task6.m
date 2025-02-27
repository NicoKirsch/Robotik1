robot = loadrobot('abbIrb1600','DataFormat','row');

analyticalIK = analyticalInverseKinematics(robot)
showdetails(analyticalIK)
analyticalIK.KinematicGroup

%Der Endeffektor und die Base des Roboters sind korrekt konfiguriert
generateIKFunction(analyticalIK,'robotIK');

%Positionen definieren
eePosition = [-0.3 0.3 1.1];    %normale Position
eePosition1 = [0.15 0 1.7];     %nahe einer Singularität
eePosition2 = [0.15 0 1.787];   %Singularität

%Transformationsmatrix berechnen
eePose = trvec2tform(eePosition);
eePose1 = trvec2tform(eePosition1);
eePose2 = trvec2tform(eePosition2);

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

%Anzeigen der berechneten Konfigurationen
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


