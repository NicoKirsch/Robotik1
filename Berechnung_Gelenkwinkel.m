robotParams.L1 = 2;  % Länge des ersten Glieds
robotParams.L2 = 2;  % Länge des zweiten Glieds


targetPosition = [2.0;3.0 ];  % Beispielhafte Zielposition
jointAngles = inverseKinematics(robotParams, targetPosition);
(jointAngles*360)/(2*pi)