
% s(1,:) = q % Position time scaling, s(t)
% s(2,:) = qd % Velocity time scaling, ds/dt
% s(3,:) = qdd % Acceleration time scaling, d^2s/dt^2

resultierendePosition = sqrt(sum(q.^2, 1));
resultierendeGeschwindigkeit = sqrt(sum(qd.^2, 1)); % Korrekt: Summierung über Zeilen
resultierendeBeschleunigung = sqrt(sum(qdd.^2, 1));



s(1,:) = resultierendePosition % Position time scaling, s(t)
s(2,:) = resultierendeGeschwindigkeit % Velocity time scaling, ds/dt
s(3,:) = resultierendeBeschleunigung % Acceleration time scaling, d^2s/dt^2

% Normalize s
s_normalized = s ./ max(s, [], 2); 
%Startposition
T0 = trvec2tform(q(:,1)')

%Zielposition
TF = trvec2tform(q(:,40)')
t = [0 1];

tvec = 1:1:50;


hilfspunkte = [ 0.5  0.5  1.2;
              0.5  0    1;
              0.5  -0.5 1.2]';


[tfInterp, v1, a1] = transformtraj( T0,TF,t,tvec, "TimeScaling",s_normalized);

% rotations = tform2quat(tfInterp);
% translations = tform2trvec(tfInterp);
% 
% plotTransforms(translations,rotations)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')


jointAngles = zeros(6, numberOfSamples);

%% Trajectory following loop
for idx = 1:numberOfSamples
    % Solve IK
    tgtPose = tfInterp(:,:,idx)
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
