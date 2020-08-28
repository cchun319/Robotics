function [joint_positions] = joint_position(q_new)
%writer: guojin li
%date: 2018.10.14
% usage: this function is to find joint positions in workspace
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q_new=[q_new,0,0];
[jointPositions,T0e] = calculateFK_sol(q_new);
joint_positions=jointPositions;
joint6=T0e(1:3,4);
joint6=joint6';
joint_positions=[joint_positions;joint6];
%Outputs the 6x3 of the locations of each joint in the Base Frame
end
