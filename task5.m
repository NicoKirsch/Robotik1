robot = importrobot('abbIrb1600.urdf')
type abbIrb1600.urdf
robotRBT = loadrobot("abbIrb1600")
homeConfig = homeConfiguration(robot);

%inverse Kinematic definieren
ik = inverseKinematics("RigidBodyTree",robotRBT)
weights = [0 0 0 1 1 1];

%Positionen definieren
pos = [-0.3 0.3 1.1];%nicht in einer Singularität
pos1 = [0.15 0 1.58]; % nahe einer Singularität
pos2 = [0.8150 0 0.9615]; % Betriebsanleitung S.56 Position 1
%1627
%Transformationsmatrix berechnen
poseTF = trvec2tform(pos);
poseTF1 = trvec2tform(pos1);
poseTF2 = trvec2tform(pos2);

% normale Position
[avgTimerVal, avgIterations] = berechneGelenkpositionen(ik, poseTF, weights, homeConfig);
fprintf('Durchschnittliche Zeit (normale Position): %f Sekunden\n', avgTimerVal);
fprintf('Durchschnittliche Iterationen (normale Position): %f\n', avgIterations);

% nahe einer Singularität
[avgTimerVal1, avgIterations1] = berechneGelenkpositionen(ik, poseTF1, weights, homeConfig);
fprintf('Durchschnittliche Zeit (nahe einer Singularität): %f Sekunden\n', avgTimerVal1);
fprintf('Durchschnittliche Iterationen (nahe einer Singularität): %f\n', avgIterations1);

% Singularität
[avgTimerVal2, avgIterations2] = berechneGelenkpositionen(ik, poseTF2, weights, homeConfig);
fprintf('Durchschnittliche Zeit (Singularität): %f Sekunden\n', avgTimerVal2);
fprintf('Durchschnittliche Iterationen (Singularität): %f\n', avgIterations2);

function [avgTimerVal, avgIterations] = berechneGelenkpositionen(ik, poseTF, weights, homeConfig)
    % berechnet die Gelenkpositionen und gibt die durchschnittliche Zeit und Iterationen zurück.
    timerVal = 0;
    iterations = 0;
    
    for i = 1:100
        tic
        [configSoln, solnInfo] = ik("tool0", poseTF, weights, homeConfig);
        timerNow = toc;
        timerVal = timerVal + timerNow;
        iterations = iterations + solnInfo.Iterations;
    end
    
    avgTimerVal = timerVal / 100;
    avgIterations = iterations / 100;
end
