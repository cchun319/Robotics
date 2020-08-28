function [q_joints] = get_q(startpoint,obstacle)
% randomly generate q 
joint1_lower_limit = -140;
joint1_upper_limit = 140;
joint2_lower_limit = -120;
joint2_upper_limit = 140;
joint3_lower_limit = -180;
joint3_upper_limit = 180;
joint4_lower_limit = -190;
joint4_upper_limit = 170;
joint5_lower_limit = -200;
joint5_upper_limit = 150;
error = 40;
while 1
q = double([(randi([joint1_lower_limit, joint1_upper_limit])), (randi([joint2_lower_limit, joint2_upper_limit])), (randi([joint3_lower_limit, joint3_upper_limit])), (randi([joint4_lower_limit, joint4_upper_limit])), (randi([joint5_lower_limit, joint5_upper_limit])), 0,0]) / 100;
qcheck=q(1,1:4);
check_output=check_collision(obstacle,qcheck);
  if(check_output==0)
  [jointPositions,T0e] = calculateFK_sol(q);
  dis=(T0e(1,4)-startpoint(1)).^2+(T0e(2,4)-startpoint(2)).^2+(T0e(3,4)-startpoint(3)).^2;
  if dis<(error*error)
      q_joints=[q(1),q(2),q(3),q(4)];
      break;
  end
  end
end
end
