function [q_is_possible] = IK_lynx_Chun3(T0e)
% Input:    T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)

% Outputs:  q - a 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) which
%               are required for the Lynx robot to reach the given 
%               transformation matrix T
% 
%           is_possible - a boolean set to true if the provided
%               transformation T is achievable by the Lynx robot, ignoring
%               joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

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

t4 = atan2( T36(2,3), T36(1,3));
t5 = atan2( - T36(3,1), - T36(3,2));

if (t1<-1.40 || t1>1.40)
        disp('not executable angle for joint 1 since beyond the limit ')
    return
end

if (x == 0 && y == 0)
    disp('inf')
    return
end

if (t3<-1.80 || t3>1.80)
    disp('not executable angle for joint 3 since beyond the limit')
    return 
end

if (t2<-1.20 || t2>1.40)
   disp('not executable angle for joint 2 since beyond the limit')
   return 
end

if (t4<-1.90 || t4>1.70)
    disp('not executable angle for joint 4 since beyond the limit')
    return 
end

if (t5<-2.00 || t5>1.50)
    disp('not executable angle for joint 5 since beyond the limit')
    return 
end

q_is_possible = [t1, t2, t3, t4, t5];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end