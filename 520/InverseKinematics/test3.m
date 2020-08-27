T0e = [          0.8266    0.5619    0.0317  -51.3357
    0.5574   -0.8096   -0.1838  297.6382
   -0.0776    0.1696   -0.9825   52.6212
         0         0         0    1.0000];
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper
a1 = 0;
d2 = 0;
d3 = 0;

alpha1 = - pi / 2;
alpha2 = 0;
alpha3 = 0;

d6 = d5 + lg;

x = T0e(1,4) - d6 * T0e(1,3);
y = T0e(2,4) - d6 * T0e(2,3);
z = T0e(3,4) - d6 * T0e(3,3);

l3 = sqrt(x^2 + y^2 +(z - d1)^2);

t1 = atan2(y, x);
t1b = atan2(y, x) + pi;
D = ( a2^2 + a3^2 - l3^2 ) / (2 * a2 * a3); % law of cos to get (theta3)
t3 = atan2(D , sqrt(1 - D^2));
t3b = atan2(D, -sqrt(1 - D^2));
B = ( a2^2 + l3^2 - a3^2 ) / (2 * a2 * l3); % law of cos to get (beta) 
Beta = atan2(sqrt(1- B^2), B);
Betab = atan2(-sqrt(1- B^2), B);

alpha = atan2((z - d1), sqrt( x^2 + y^2 ));

t2 = pi/2 - alpha - Beta;
t2b = pi/2 - alpha - Betab;

t2c = pi*3/2 + alpha + Betab;
t2d = pi*3/2 + alpha + Beta;

theta1 = t1;
theta2 = t2 - pi/2;
theta3 = t3 + pi/2;

theta1b = t1b;
theta2b = t2b - pi/2;
theta2c = t2c - pi/2;
theta2d = t2d - pi/2;
theta3b = t3b + pi/2;

T011 = [cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) a1*cos(theta1);
    sin(theta1) cos(alpha1)*cos(theta1) -sin(alpha1)*cos(theta1) a1*sin(theta1);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];

T121 = [cos(theta2) -cos(alpha2)*sin(theta2) sin(alpha2)*sin(theta2) a2*cos(theta2);
    sin(theta2) cos(alpha2)*cos(theta2) -sin(alpha2)*cos(theta2) a2*sin(theta2);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];

T231 = [cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3) a3*cos(theta3);
    sin(theta3) cos(alpha3)*cos(theta3) -sin(alpha3)*cos(theta3) a3*sin(theta3);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];

T012 = [cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) a1*cos(theta1);
    sin(theta1) cos(alpha1)*cos(theta1) -sin(alpha1)*cos(theta1) a1*sin(theta1);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];

T122 = [cos(theta2b) -cos(alpha2)*sin(theta2b) sin(alpha2)*sin(theta2b) a2*cos(theta2b);
    sin(theta2b) cos(alpha2)*cos(theta2b) -sin(alpha2)*cos(theta2b) a2*sin(theta2b);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];

T232 = [cos(theta3b) -cos(alpha3)*sin(theta3b) sin(alpha3)*sin(theta3b) a3*cos(theta3b);
    sin(theta3b) cos(alpha3)*cos(theta3b) -sin(alpha3)*cos(theta3b) a3*sin(theta3b);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];

T013 = [cos(theta1b) -cos(alpha1)*sin(theta1b) sin(alpha1)*sin(theta1b) a1*cos(theta1b);
    sin(theta1b) cos(alpha1)*cos(theta1b) -sin(alpha1)*cos(theta1b) a1*sin(theta1b);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];

T123 = [cos(theta2c) -cos(alpha2)*sin(theta2c) sin(alpha2)*sin(theta2c) a2*cos(theta2c);
    sin(theta2c) cos(alpha2)*cos(theta2c) -sin(alpha2)*cos(theta2c) a2*sin(theta2c);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];

T233 = [cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3) a3*cos(theta3);
    sin(theta3) cos(alpha3)*cos(theta3) -sin(alpha3)*cos(theta3) a3*sin(theta3);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];

T014 = [cos(theta1b) -cos(alpha1)*sin(theta1b) sin(alpha1)*sin(theta1b) a1*cos(theta1b);
    sin(theta1b) cos(alpha1)*cos(theta1b) -sin(alpha1)*cos(theta1b) a1*sin(theta1b);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];

T124 = [cos(theta2d) -cos(alpha2)*sin(theta2d) sin(alpha2)*sin(theta2d) a2*cos(theta2d);
    sin(theta2d) cos(alpha2)*cos(theta2d) -sin(alpha2)*cos(theta2d) a2*sin(theta2d);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];

T234 = [cos(theta3b) -cos(alpha3)*sin(theta3b) sin(alpha3)*sin(theta3b) a3*cos(theta3b);
    sin(theta3b) cos(alpha3)*cos(theta3b) -sin(alpha3)*cos(theta3b) a3*sin(theta3b);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];
          
%q1 = [t1, t2, t3, 0 ,0]
%q2 = [t1, t2b, t3b, 0 ,0]
%q3 = [t1b, t2c, t3 ,0 , 0 ]
%q4 = [t1b, t2d, t3b ,0 , 0 ]

T031 = T011*T121*T231;
T032 = T012*T122*T232;
T033 = T013*T123*T233;
T034 = T014*T124*T234;

T361 = inv(T031) * T0e;
T362 = inv(T032) * T0e;
T363 = inv(T033) * T0e;
T364 = inv(T034) * T0e;

t41 = atan2( T361(2,3), T361(1,3));
t51 = atan2( - T361(3,1), - T361(3,2));
t42 = atan2( T362(2,3), T362(1,3));
t52 = atan2( - T362(3,1), - T362(3,2));
t43 = atan2( T363(2,3), T363(1,3));
t53 = atan2( - T363(3,1), - T363(3,2));
t44 = atan2( T364(2,3), T364(1,3));
t54 = atan2( - T364(3,1), - T364(3,2));

q1 = [t1, t2, t3, t41 ,t51]
q2 = [t1, t2b, t3b, t42 ,t52]
q3 = [t1b, t2c, t3 ,t43 ,t53]
q4 = [t1b, t2d, t3b ,t44 , t54]

[jointPositions,T0e1] = calculateFK_sol(q1)
[jointPositions,T0e2] = calculateFK_sol(q2)
[jointPositions,T0e3] = calculateFK_sol(q3)
[jointPositions,T0e4] = calculateFK_sol(q4)