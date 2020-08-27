function [Jv2] = calcJacobianjoint(q)

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 85; %wrist to base of gripper
lg = 15; %length of gripper

% relevant info from position FK
[T0] = calculateFK_transforms(q);



%% LINEAR VELOCITY JACOBIAN
% Approach: cross products
Jv2 = zeros(3,5,5);
for i2 = 1:1
    Jv2(:,i2,1) = cross(T0(1:3,3,i2), T0(1:3,4,2)-T0(1:3,4,i2));
end
% J2=Jv2;
% 
% Jv3 = zeros(3,5);
for i3 = 1:2
    Jv2(:,i3,2) = cross(T0(1:3,3,i3), T0(1:3,4,3)-T0(1:3,4,i3));
end
% J3=Jv3;
% 
% Jv4 = zeros(3,5);
for i4 = 1:3
    Jv2(:,i4,3) = cross(T0(1:3,3,i4), T0(1:3,4,4)-T0(1:3,4,i4));
end
% J4=Jv4;
% 
% 
% Jv5 = zeros(3,5);
for i5 = 1:4
    Jv2(:,i5,4) = cross(T0(1:3,3,i5), T0(1:3,4,5)-T0(1:3,4,i5));
end
% J5=Jv5;
% 
% Jv6 = zeros(3,5);
for i6 = 1:5
    Jv2(:,i6,5) = cross(T0(1:3,3,i6), T0(1:3,4,end)-T0(1:3,4,i6));
end
% J6=Jv6;
%% Compose Jacobian Matrix and calc end effector velocities




%% SUPPORTING FUNCTIONS
    function [T0] = calculateFK_transforms(q)
        % Input: q - 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5]
        
        % Outputs:  T0 = 6 transformation matrices T_i^0 for transforming from frame
        %                 i-1 to frame 0
        %           Tint = 5 transformation matrices T_i^i-1 for
        %                 transforming between intermediate frames
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        % CODE TAKEN FROM calculateFK_sol %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %Frame 1 w.r.t Frame 0
        T1 = [cos(q(1))  0  -sin(q(1))  0;
            sin(q(1))  0   cos(q(1))  0;
            0    -1       0      d1;
            0     0       0      1];
        
        %Frame 2 w.r.t Frame 1
        T2 = [sin(q(2)) cos(q(2))  0   a2*sin(q(2));
            -cos(q(2))  sin(q(2))  0   -a2*cos(q(2));
            0              0       1          0;
            0              0       0          1];
        
        %Frame 3 w.r.t Frame 2
        T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
            sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
            0                        0  1                     0;
            0                        0  0                     1];
        
        %Frame 4 w.r.t Frame 3
        T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
            sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
            0                          sin(-pi/2)                    cos(-pi/2)   0;
            0                                   0                             0   1];
        %Frame 5 w.r.t Frame 4
        T5 = [cos(q(5)) -sin(q(5))  0        0;
            sin(q(5))  cos(q(5))  0        0;
            0          0  1       d5 + lg;
            0          0  0        1];
        
        % NEW CODE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        T0 = eye(4);
        T0(:,:,2) = T1;
        T0(:,:,3) = T1*T2;
        T0(:,:,4) = T1*T2*T3;
        T0(:,:,5) = T1*T2*T3*T4;
        T0(:,:,6) = T1*T2*T3*T4*T5;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

end
