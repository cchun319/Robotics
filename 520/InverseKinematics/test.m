T = [  0.0000   -0.7071    0.7071   89.4370
   -0.0000    0.7071    0.7071  -89.4370
   -1.0000   -0.0000    0.0000  -38.1000
         0         0         0    1.0000];

T2 = [ 0.7006    0.5090    0.5000   52.3875
   -0.4045   -0.2939    0.8660   90.7378
    0.5878   -0.8090    0.0000    0.0000
         0         0         0    1.0000]     
% 
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

theta1 = atan2( - T(1,3), - T(2,3));
theta2 = acos( (T(1,4) - a3 * T(1,1)) / (a2 * cos(theta1)));
theta3 = atan2( T(3,1), T(3,2)) - theta2;

theta5 = atan2( - T2(3,1), - T2(3,2));
theta4 = atan2( - T2(1,3), - T2(2,3));

theta1
theta2 = theta2 + pi/2
theta3 = theta3 - pi/2
theta4 = theta4 + pi/2
theta5