%function [q, is_pos] = get_q2(pos)

% joint1_lower_limit = -140;
% joint1_upper_limit = 140;
% joint2_lower_limit = -120;
% joint2_upper_limit = 140;
% joint3_lower_limit = -180;
% joint3_upper_limit = 180;
% joint4_lower_limit = -190;
% joint4_upper_limit = 170;
% joint5_lower_limit = -200;
% joint5_upper_limit = 150;
% error = 100;

pos = [280 0 250]

for i = 1 : 50000
    q_gus = ([(randi([-14000, 14000])), (randi([-12000, 14000])), (randi([-18000, 18000])), (randi([-19000, 17000])), 0,0]) / 10e5;
    [j_gus, T0e_gus] = calculateFK_sol(q_gus);
    T0e_gus(1:3, 4) = pos';
    [q_yes, is_possible] = IK_lynx_sol(T0e_gus);
    if is_possible
        q = q_yes;
        [j_yes, T0e_yes] = calculateFK_sol(q);
        j_yes(6,: )
        break
    else if i == 50000
            error('fail')
        end
    end
end

