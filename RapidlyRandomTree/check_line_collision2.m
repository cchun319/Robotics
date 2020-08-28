function [line_bool2] = check_line_collision2(obstacle,q_close, q_new)
    
    interpolate = 100;
    
    v = double(q_new - q_close);
        
    dist = norm(v);
    u = v / dist;
    step_check = dist / interpolate;
    
    now_q = double(q_close);
    
    for i = 1 : interpolate
        now_q = now_q + (step_check * u);
        if check_collision(obstacle,now_q) == 1
            line_bool2 = 1;
        else
            line_boo12 = 0;
        end
end