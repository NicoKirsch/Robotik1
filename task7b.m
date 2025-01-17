
s(1,:) = q % Position time scaling, s(t)
s(2,:) = qd % Velocity time scaling, ds/dt
s(3,:) = qdd % Acceleration time scaling, d^2s/dt^2

T0 = trvec2tform(q(:,1)');
TF = trvec2tform(q(:,numberOfSamples)');
t = [0 10]

transformtraj( T0,TF,t,"TimeScaling",s)