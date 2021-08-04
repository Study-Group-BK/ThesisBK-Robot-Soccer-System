z=tf('z');
Ts=0.2;
Kp=25.25
Ki=0.7

Gz=Kp+Ts*Ki*1/(z-1)
Gz.variable='z^-1'


