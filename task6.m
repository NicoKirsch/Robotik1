
%task6

robot = loadrobot('abbIrb1600','DataFormat','row');
show(robot);

analyticalIK = analyticalInverseKinematics(robot)
showdetails(analyticalIK)
analyticalIK.KinematicGroup
%Der Endeffektor und die Base des Roboters sind korrekt konfiguriert
generateIKFunction(analyticalIK,'robotIK');

%Anzeigen der gewünschten Endposition
%eePosition = [-0.3 0.3 1.1];
eePosition = [0.15 0 1.7];%in einer Singularität
%eePosition = [0.15 0 1.787];%in einer Singularität
eePose = trvec2tform(eePosition);
hold on
plotTransforms(eePosition,tform2quat(eePose))
hold off

tic
    ikConfig = robotIK(eePose); % Uses the generated file
toc

figure;

numSolutions = size(ikConfig,1)


for i = 1:size(ikConfig,1)
    subplot(1,numSolutions,i)
    show(robot,ikConfig(i,:));
end
