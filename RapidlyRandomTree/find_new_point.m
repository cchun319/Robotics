function [q_new] = find_new_point(q_close, q_rand, step)
%writer: guojin li
%date: 2018.10.14
% usage: this function is to find a new point in map
% the random point is just for direction, we decide the step length by
%arameter step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    direction = double(q_rand - q_close);
    magnitude=norm(direction);
    u = direction / magnitude;
    q_new = double(q_close + step * u);
    
end