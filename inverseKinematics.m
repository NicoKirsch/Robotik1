function jointAngles = inverseKinematics2D(robotParams, targetPosition)
% INVERSEKINEMATICS2D Berechnet die Gelenkwinkel eines 3-DOF-Roboters in der x-y-Ebene.
%
%   jointAngles = INVERSEKINEMATICS2D(robotParams, targetPosition) berechnet die 
%   Gelenkwinkel eines 3-DOF-Roboters mit den gegebenen Parametern und der 
%   gewünschten Zielposition in der x-y-Ebene.
%
%   Eingabe:
%       robotParams: Struktur mit den Roboterparametern (z.B. L1, L2)
%       targetPosition: 2x1-Vektor mit der x, y-Zielposition des Endeffektors
%
%   Ausgabe:
%       jointAngles: 2x1-Vektor mit den berechneten Gelenkwinkeln theta1 und theta2

% Entpacke die Roboterparameter
l1 = robotParams.L1;
l2 = robotParams.L2;
x_d = targetPosition(1);
y_d = targetPosition(2);

r = sqrt(x_d^2+y_d^2);
cos_theta2 = (r^2-l1^2-l2^2)/(2*l1*l2);
sin_theta2 = sqrt(1-cos_theta2^2);

theta2_1 = atan2(sin_theta2,cos_theta2)
theta2_2 = atan2(-sin_theta2,cos_theta2)

theta1_1 = atan2(y_d,x_d) - atan2(l2*sin(theta2_1),l1+l2*cos(theta2_1))
theta1_2 = atan2(y_d,x_d) - atan2(l2*sin(theta2_2),l1+l2*cos(theta2_2))

jointAngles = [theta1_1,theta1_2;theta1_2,theta2_1];
end