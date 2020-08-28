 function [q_is_possible,solution] = IK_lynx_liguojinchangchun(T0e)
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
d6=100; % this is d5+lg=85+15mm=100mm 
d1=68.0;  %linkage 1 length
a2=147.0; %linkage 2 length 
a3=187.0;  
d5=85;
lg=15;
R=T0e(1:3,1:3);  %extract whole rotation matix
o=T0e(1:3,4);    %extract end effector coordinate in base frame  it is a colume
o=o';   %transform the position vector to be a row 
% o=[x;y;z]; R input  %this is the end-effector positions 
c(1)=o(1)-d6*R(1,3);
c(2)=o(2)-d6*R(2,3);
c(3)=o(3)-d6*R(3,3); %c represents the wrist position in the base frame
x=c(1);   % the x coordinate of wrist in base frame
y=c(2);   % the y coordinate of wrist in base frame
z=c(3);   % the z coordinate of wrist in base frame
s=z-d1;   % to compute psi angle 
r=sqrt(x*x+y*y);
% compute q1 q2 q3
if (x==0 && y==0)   
    disp('infinite values for q1');
% if x=y=0, there will be infinite solutions for theta 1
 if (q1<-1.40 || q1>1.40)
   disp('not executable angle for joint 1 since beyond the limit ')
   solution=0; 
   q_is_possible=0;
   return
    end
else
     q1=atan(y/x);
end
% there is only one solution for q1 since q1 should not exceed joint limit
% now find the solution for thera 3
D=(a2*a2+a3*a3-r*r-s*s)/(2*a2*a3);
q3=atan(D/sqrt(1-D*D));
% joint limit for theta 3
if (q3<-1.80 || q3>1.80)
    disp('not executable angle for joint 3 since beyond the limit');
    solution=0;
    q_is_possible=0;
    return 
end
%according to different situation, using different expression for theta 2
if x>=0
psi=atan2(s,r);
q2=pi/2-psi-atan(a3*cos(q3)/(a2-a3*sin(q3)));
else
psi=atan2(s,r);
q2=pi/2-(pi-psi)-atan(a3*cos(q3)/(a2-a3*sin(q3)));
end
%joint limit for theta 2
 if (q2<-1.20 || q2>1.40)
    disp('not executable angle for joint 2 since beyond the limit');
    solution=0;
    q_is_possible=0;
    return 
 end
% DH parameter for joint 1 2 and 3
alpha=[-pi/2,0,0];
a=[0,a2,a3];
d=[d1,0,0];
%compute A1
alpha1=alpha(1);
a1=a(1);
d1=d(1);
theta1=q1;
A1=[cos(theta1) (-1).*(sin(theta1)).*(cos(alpha1)) (sin(theta1)).*(sin(alpha1)) a1.*(cos(theta1));
sin(theta1) (cos(theta1)).*(cos(alpha1)) (-1).*(cos(theta1).*(sin(alpha1))) (sin(theta1)).*(a1);
0 sin(alpha1) cos(alpha1) d1;0 0 0 1];
% compute A2
alpha2=alpha(2);
a2=a(2);
d2=d(2);
theta2=q2-pi/2;
A2=[cos(theta2) (-1).*(sin(theta2)).*(cos(alpha2)) (sin(theta2)).*(sin(alpha2)) a2.*(cos(theta2));
sin(theta2) (cos(theta2)).*(cos(alpha2)) (-1).*(cos(theta2).*(sin(alpha2))) (sin(theta2)).*(a2);
0 sin(alpha2) cos(alpha2) d2;0 0 0 1];
% compute A3
alpha3=alpha(3);
a3=a(3);
d3=d(3);
theta3=q3+pi/2;
A3=[cos(theta3) (-1).*(sin(theta3)).*(cos(alpha3)) (sin(theta3)).*(sin(alpha3)) a3.*(cos(theta3));
sin(theta3) (cos(theta3)).*(cos(alpha3)) (-1).*(cos(theta3).*(sin(alpha3))) (sin(theta3)).*(a3);
0 sin(alpha3) cos(alpha3) d3;0 0 0 1];
% compute homogeneous transformation matrix from base frame to the third coordinate frame 
T03=A1*A2*A3;
% extract rotation matrix R03
R03(1,:)=T03(1,1:3);
R03(2,:)=T03(2,1:3);
R03(3,:)=T03(3,1:3);
% compute R36
R36=(R03)'*R;
% find q4 q5;
q4=atan(R36(2,3)/R36(1,3));
% joint limit for theta 4
if (q4<-1.90 || q4>1.70)
    disp('not executable angle for joint 4 since beyond the limit');
    solution=0;
    q_is_possible=0;
    return 
end
q5=atan(R36(3,1)/R36(3,2));
% joint limit fot theta 5
if (q5<-2.00 || q5>1.50)
disp('not executable angle for joint 5 since beyond the limit');
solution=0;
q_is_possible=0;
return 
end
q_is_possible=[q1,q2,q3,q4,q5,0];
solution=1;
% run the robot to this possible solution 
% lynxServo(q_is_possible)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end