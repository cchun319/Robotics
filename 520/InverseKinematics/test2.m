T0e = [       0.1593    0.7602   -0.6299 -151.6017
   -0.6464   -0.4019   -0.6485 -156.0950
   -0.7462    0.5105    0.4274  360.7568
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

t1 = atan(y/ x);
D = ( a2^2 + a3^2 - l3^2 ) / (2 * a2 * a3);
t3 = atan2(D, sqrt(1 - D^2));
beta = atan2((a3 * sin( pi/2 - t3 )), (a2 - a3 * cos(pi/2 - t3)));
if x > 0 
    alpha = atan2(z - d1, sqrt( x^2 + y^2 ));
    t2 = pi/2 - alpha - beta;
else
    alpha = atan2(sqrt( x^2 + y^2 ),z - d1);
    t2 = - ( alpha + beta);
end

theta1 = t1;
theta2 = t2 - pi/2;
theta3 = t3 + pi/2;

T01 = [cos(theta1) -cos(alpha1)*sin(theta1) sin(alpha1)*sin(theta1) a1*cos(theta1);
    sin(theta1) cos(alpha1)*cos(theta1) -sin(alpha1)*cos(theta1) a1*sin(theta1);
    0 sin(alpha1) cos(alpha1) d1;
    0 0 0 1];

T12 = [cos(theta2) -cos(alpha2)*sin(theta2) sin(alpha2)*sin(theta2) a2*cos(theta2);
    sin(theta2) cos(alpha2)*cos(theta2) -sin(alpha2)*cos(theta2) a2*sin(theta2);
    0 sin(alpha2) cos(alpha2) d2;
    0 0 0 1];

T23 = [cos(theta3) -cos(alpha3)*sin(theta3) sin(alpha3)*sin(theta3) a3*cos(theta3);
    sin(theta3) cos(alpha3)*cos(theta3) -sin(alpha3)*cos(theta3) a3*sin(theta3);
    0 sin(alpha3) cos(alpha3) d3;
    0 0 0 1];


T03 = T01 * T12 * T23;
T36 = inv(T03) * T0e;

t1;
t2;
t3;
t4 = atan2( T36(2,3), T36(1,3));
t5 = atan2( - T36(3,1), - T36(3,2));
t1, t2, t3, t4, t5 
