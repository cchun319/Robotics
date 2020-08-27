function [q] = rand_move(q_cool, obstacle)
ratio=0.005;

lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15];
upperLim = [1.4 1.4 1.7 1.7 1.5 30];
q_random = [0 0 0 0 0];
limit = 100000;
% for k1 = 1 : limit
%     q_random=[ q_cool(end,1)+ratio(1)*k1*randi([-1,1]), q_cool(end,2)+ratio(1)*k1*randi([-1,1]), q_cool(end,3)+ratio(1)*k1*randi([-1,1]), q_cool(end,4)+ratio(1)*k1*randi([-1,1]), q_cool(end,5)+ratio(1)*k1*randi([-1,1]), 0];
%     [bool] = check_collision(obstacle,q_random);
%     if (q_random(1) > lowerLim(1) && q_random(1) < upperLim(1)) && (q_random(2) > lowerLim(2) && q_random(2) < upperLim(2))&& (q_random(3) > lowerLim(3) && q_random(3) < upperLim(3))&& (q_random(4) > lowerLim(4) && q_random(4) < upperLim(4))...
%             &&(q_random(5) > lowerLim(5) && q_random(5) < upperLim(5))&& (bool == 0)
%         q = q_random;
%         break
%     elseif k1 == limit
%         error('no viable q');
%     end
% end

for k1 = 1 : limit
    ratio = ratio + 0.0001;
    
    for i=1:5
        if rand(1)>=0.5
            q_random(i) = q_cool(end,i) + ratio;
        else
            q_random(i) = q_cool(end,i) - ratio;
        end
    end
    [bool] = check_collision(obstacle,q_random);
    [j_cool,T0e_cool] = calculateFK_sol(q_cool(end, :));
    [j_rand,T0e_rand] = calculateFK_sol([q_random, 0]);
    if norm(j_cool(6,:) - j_rand(6,:)) < 80
        if (q_random(1) > lowerLim(1) && q_random(1) < upperLim(1)) && (q_random(2) > lowerLim(2) && q_random(2) < upperLim(2))&& (q_random(3) > lowerLim(3) && q_random(3) < upperLim(3))&& (q_random(4) > lowerLim(4) && q_random(4) < upperLim(4))...
                (q_random(5) > lowerLim(5) && q_random(5) < upperLim(5)) && (bool == 0)
            q = [q_random,0];
            break
        end
    end
    if k1 == limit
        error('no viable q');
    end
end
end