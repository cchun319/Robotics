% q_close <--> q_goal <--> q_new
function [qgoalontheline, q_ideal] = qgoal_on_the_line(q_close, q_new, q_goal)
    diection = double(q_new - q_close);
    magnitude = norm(diection);
    u = diection / magnitude;
    qgoalontheline = norm(double(q_goal - q_close));
    q_ideal = [ 0 0 0 0];
    if qgoalontheline > magnitude
        qgoalontheline = 0;
        return;
    end
    q_goal_ideal = double(q_close) + (qgoalontheline * u);
        
    jointpositions_ideal = joint_position(q_goal_ideal);
    joint6_ideal = jointpositions_ideal( 7, :);
    jointpositions_goal = joint_position(q_goal);
    joint6_goal = jointpositions_goal( 7, :);
    
    p = [joint6_ideal; joint6_goal];
    dist_final = pdist(p, 'euclidean');
    
    qgoalontheline = (isequal(double(q_goal_ideal(4)), q_goal(4)) && ( dist_final < 100 ));
    
    if qgoalontheline == 1
        q_ideal = q_goal_ideal;
    end
    
    %qgoalontheline = (isequal(double(q_goal_ideal(1)), q_goal(1)))&& ( dist_final < 10 );
    %qgoalontheline = (isequal(double(q_goal_ideal(3)), q_goal(3)))&&(isequal(double(q_goal_ideal(4)), q_goal(4))) && ( dist_final < 100 );
    %qgoalontheline = (isequal(double(q_goal_ideal(1)), q_goal(1)))&&(isequal(double(q_goal_ideal(2)), q_goal(2))) && ( dist_final < 10 );
    %qgoalontheline = (isequal(double(q_goal_ideal(3)), q_goal(3)))&&(isequal(double(q_goal_ideal(4)), q_goal(4))) ;
    %qgoalontheline = (isequal(double(q_goal_ideal(3)), q_goal(3)))&&(isequal(double(q_goal_ideal(4)), q_goal(4)))&&(isequal(double(q_goal_ideal(2)), q_goal(2))) ;
    %qgoalontheline = isequal(q_goal_ideal,q_goal);
end
