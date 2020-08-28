function e_vel = FK_velocity(q, qdot)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]

% Outputs:  e_vel - 6 x 1 vector of end effector velocities, where
%                    e_vel(1:3) are the linear velocity
%                    e_vel(4:6) are the angular velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Your code here

theta = [q(1) q(2)-pi/2 q(3)+pi/2 q(4)-pi/2 q(5) q(6)];
alpha = [-pi/2 0 0 -pi/2 0 0];
a = [0 146.05 187.325 0 0 28.575];
d = [76.2 0 0 0 76.2 0];

l1 = 76.2; % base height (table to center of joint 2)
l2 = 146.05; % shoulder to elbow length
l3 = 187.325; %elbow to wrist length
l4 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

T01 = [cos(theta(1)) 0 sin(theta(1))*-1 a(1)*cos(theta(1));
    sin(theta(1)) 0 -cos(theta(1))*-1 a(1)*sin(theta(1));
    0 -1 0 d(1);
    0 0 0 1];

T12 = [sin(q(2)) cos(q(2))*1 0 a(2)*sin(q(2));
    -cos(q(2)) sin(q(2))*1 0 a(2)*-cos(q(2));
    0 0 1 d(2);
    0 0 0 1];

T23 = [-sin(q(3)) -cos(q(3))*1 0 a(3)*-sin(q(3));
    cos(q(3)) -sin(q(3))*1 0 a(3)*cos(q(3));
    0 0 1 d(3);
    0 0 0 1];

T34 = [sin(q(4)) 0 -cos(q(4))*-1 a(4)*sin(q(4));
    -cos(q(4)) 0 -sin(q(4))*-1 a(4)*-cos(q(4));
    0 -1 0 d(4);
    0 0 0 1];

T45 = [cos(theta(5)) -sin(theta(5))*1 0 a(5)*cos(theta(5));
    sin(theta(5)) cos(theta(5))*1 0 a(5)*sin(theta(5));
    0 0 1 d(5);
    0 0 0 1];

T56 = [1 0 0 0;
    0 1 0 0;
    0 0 1 lg;
    0 0 0 1];
T01;
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;

T_s=[0 0 0]';

O0 = T06(1:3,4) - T_s;
O1 = T06(1:3,4) - T01(1:3,4);
O2 = T06(1:3,4) - T02(1:3,4);
O3 = T06(1:3,4) - T03(1:3,4);
O4 = T06(1:3,4) - T04(1:3,4);
O5 = T06(1:3,4) - T05(1:3,4);

T00=[0 0 1]';

%get jacobain by cross product

j = [cross(T00, O0) cross(T01(1:3,3), O1) cross(T02(1:3,3), O2) cross(T03(1:3,3), O3) cross(T04(1:3,3), O4) cross(T05(1:3,3), O5);
            T00 T01(1:3,3) T02(1:3,3) T03(1:3,3) T04(1:3,3) T05(1:3,3)]; 

e_vel = round(((j) * qdot'), 2);
end