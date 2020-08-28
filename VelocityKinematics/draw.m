% q=[0,0,0,0,-pi/2,0];
% qdot=[1,1,1,1,1,0];
% e_vel = FK_velocity(q, qdot)
% qdot = IK_velocity(q, e_vel)

% joint limit
% [-1.4 -1.2 -1.8 -1.9 -2];
% [1.4 1.4 1.7 1.7 1.5]

% circle
lynxStart();
q=[0,0,0,0,0,0];
theta = 0;
for t=0:1:250
    % use change of tangential velocity to control the path
    e_vel=([60*cos(theta)*2*pi/250,0,-60*sin(theta)*2*pi/250,0,0,0])';
    qdot = IK_velocity(q, e_vel)
    q = q + (qdot)' * 1;
    theta = theta + 2*pi/250 * 1
    [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
    endeffect=(T0e(1:3,4))';
    lynxServo(q);
    scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
    hold on
end

% straight
% lynxStart();
% q=[0,0,0,0,0,0];
% e_vel=([-1,1,0,0,0.01,0])';
% q_list = double.empty(0, 6);
% qdot_list = double.empty(0, 6);
% for t=0:1:500
%     qdot = IK_velocity(q, e_vel)
%     q = q + (qdot)' * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05] = calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     lynxServo(q);
%     q_list = [ q_list; q];
%     qdot_list = [qdot_list;qdot'];
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end

% all same speed
% lynxStart();
% q=[0,0,0,-pi/2,0,0];
% qdot=[0.01,0.01,0.01,0.01,0.01,0];
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end



% from lower limit
% lynxStart();
% q=[-1.4 -1.2 -1.8 -1.9 -2 0];
% qdot=[0.01,0.01,0.01,0.01,0.01,0];
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end

% lynxStart();
% q=[1.4 1.4 1.7 1.7 1.5 0];
% qdot=-[0.01,0.01,0.01,0.01,0.01,0];
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end

%move one joint1

% lynxStart();
% q=[0 0 0 0 0 0];
% qdot=[0.01 0 0 0 0 0];
% evel_list = double.empty(0,6);
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     lynxServo(q);
%     evel_list = [evel_list;e_vel'];
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end

% %move one joint2
% 
% lynxStart();
% q=[0 0 0 0 0 0];
% qdot=[0 0.01 0 0 0 0];
% evel_list = double.empty(0,6);
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     evel_list = [evel_list;e_vel']
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end
% 
% %move one joint3
% 
% lynxStart();
% q=[0 0 0 0 0 0];
% qdot=[0 0 0.01 0 0 0];
% evel_list = double.empty(0,6);
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     evel_list = [evel_list;e_vel']
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end
% 
% %move one joint4
% 
% lynxStart();
% q=[0 0 0 0 0 0];
% qdot=[0 0 0 0.01 0 0];
% evel_list = double.empty(0,6);
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     evel_list = [evel_list;e_vel'];
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end
% 
% %move one joint5
% 
% lynxStart();
% q=[0 0 0 0 0 0];
% qdot=[0 0 0 0 0.01 0];
% evel_list = double.empty(0,6);
% 
% for t=0:1:250
%     e_vel = FK_velocity(q, qdot);
%     q = q + qdot * 1;
%     [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%     endeffect=(T0e(1:3,4))';
%     evel_list = [evel_list;e_vel'];
%     lynxServo(q);
%     scatter3(T0e(1,4), T0e(2,4), T0e(3,4))
%     hold on
% end

% circle
% lynxStart();
% q=[0,0,0,0,0,0];
% theta1 = 0;
% theta2 = 0;
% r=0.1;
% for t=0:1:250
%     for i = 0:1:250
%         e_vel=([r*cos(theta2)*cos(theta1)-r*sin(theta1)*sin(theta2),r*sin(theta2)*cos(theta1)-r*sin(theta1)*cos(theta2),r*sin(theta2),0,0,0])';
%         qdot = IK_velocity(q, e_vel)
%         q = q + (qdot)' * 1;
%         theta1 = 2*pi/250 * t;
%         theta2 = pi/250 * i;
%         [jointPosition, T0e, T01, T02, T03, T04, T05]=calculateFK_sol(q);
%         endeffect=(T0e(1:3,4))';
%         lynxServo(q);
%         scatter3(T0e(1,4), T0e(2,4), T0e(3,4))      
%         hold on
%     end
% end

