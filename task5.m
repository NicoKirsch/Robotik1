robot = importrobot('abbIrb1600.urdf')
robotRBT = loadrobot("abbIrb1600")

%show(robotRBT);

homeConfig = homeConfiguration(robot);
%task 5
%inverse Kinematic Berechnung

ik = inverseKinematics("RigidBodyTree",robotRBT)

pos = [-0.3 0.3 1.1];%nicht in einer Singularität
pos1 = [0.15 0 1.7]; % nahe einer Singularität
pos2 = [0.15 0 1.787]; % Betriebsanleitung S.57 Position 1
poseTF = trvec2tform(pos);
poseTF1 = trvec2tform(pos1);
poseTF2 = trvec2tform(pos2);
weights = [0 0 0 1 1 1];

timerVal = 0;
iterations = 0;
%Berechnung der Gelenkpositionen
for i= 1:100
tic
[configSoln,solnInfo] = ik("tool0",poseTF,weights,homeConfig);
timerNow = toc;
timerVal = timerVal + timerNow;
iterations = iterations + solnInfo.Iterations;
end
avgTimerVal = timerVal/100 % Durchschnittliche Zeit zur Berechnung der Position
avgIterations = iterations/100

timerVal1 = 0;
iterations1 = 0;
%show(robotRBT,configSoln,PreservePlot=true);
%Berechnung der Gelenkpositionen
for i= 1:100
tic
[configSoln,solnInfo] = ik("tool0",poseTF1,weights,homeConfig);
timerNow = toc;
timerVal1 = timerVal1 + timerNow;
iterations1 = iterations1 + solnInfo.Iterations;
end
avgTimerVal1 = timerVal1/100 % Durchschnittliche Zeit zur Berechnung der Position
avgIterations1 = iterations1/100
%show(robotRBT,configSoln,PreservePlot=true);

timerVal2 = 0;
iterations2 = 0;
%Berechnung der Gelenkpositionen
for i= 1:100
tic
[configSoln,solnInfo] = ik("tool0",poseTF2,weights,homeConfig);
timerNow = toc;
timerVal2 = timerVal2 + timerNow;
iterations2 = iterations2 + solnInfo.Iterations;
end
avgTimerVal2 = timerVal2/100 % Durchschnittliche Zeit zur Berechnung der Position
avgIterations2 = iterations2/100

zeitlicherMehraufwand = timerVal2/timerVal
%der zeitliche Mehraufwand beträgt das 13 bis 16 fache an Zeit, wenn der
%Roboter eine Singularität anfährt(Normal ca.0.012s, Singularität ca.0.18s)

%show(robotRBT,configSoln,PreservePlot=false);
