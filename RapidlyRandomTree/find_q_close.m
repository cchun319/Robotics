function [q_close, q_close_index] = find_q_close(q_rand, T_start);

    row = size(T_start,1);
    if row < 1
        error('we cannot find a path using RRT');
    end
    
    distance = double.empty(0, 1);
    
    for j = 1 : row
        jointpositions_rand = joint_position(q_rand);
        joint6_rand = jointpositions_rand( 7, :);
        jointpositions_start = joint_position(T_start(j, :));
        joint6_start = jointpositions_start( 7, :);
        distance(j, 1) = pdist2(double(joint6_rand), double(joint6_start), 'euclidean');
        %distance(j, 1) = pdist2(double(q_rand), double(T_start(j, :)), 'euclidean');
    end
    
    min_index = find(distance == min(distance));
    
    q_close = T_start(min_index(1), :);
    
    q_close_index = min_index(1);
end