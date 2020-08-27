function [whethertoescape,q,state]=escapefromlocalminimum(q_cool,step1,thr,obstacle)
% thr=0.04;
state = double.empty(0, 1);
state = 0;
number = size(q_cool,1);

whethertoescape=0;
q=[0 0 0 0 0 0];
% threshold=0.01;
ratio=[0.01 0.01 0.01 0.01 0.01];
% if thr > step1
%     error('error selection of step');
% end
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15];
upperLim = [1.4 1.4 1.7 1.7 1.5 30];


for j=2: (number-3)
    value1=norm(q_cool(j+1,:)-q_cool(j,:));
    value2=norm(q_cool(j+2,:)-q_cool(j,:));
    value3=norm(q_cool(j+3,:)-q_cool(j,:));
    
%     if (value1<thr || value2<thr || value3<thr )
%         
%         whethertoescape=1;
%         state=[state;1];
%         
%         for k3= 1 : 500
%             for i=1:5
%                 sign(i)=rand(1);
%                 if sign(i)>=0.5
%                     for k1 = 1 : 500
%                         q_random(i)=q_cool(end,i)+ratio(i)*step1*rand(1);
%                         if (q_random(i) > lowerLim(i) && q_random(i) < upperLim(i))
%                             q(i)=q_random(i);
%                             break
%                         elseif k1 == 500
%                             error('no viable q');
%                         end
%                     end
%                     
%                 else
%                    
%                     for k2 = 1 : 500
%                         q_random(i)=q_cool(end,i)-ratio(i)*step1*rand(1);
%                         if (q_random(i) > lowerLim(i) && q_random(i) < upperLim(i))
%                             q(i)=q_random(i);
%                             break
%                         elseif k2 == 500
%                             error('no viable q');
%                         end
%                     end
%                     
%                 end
%             end
%             if(check_collision(obstacle,q)==0)
%                 break
%             elseif k3 == 500
%                 error('no viable q');
%             end
%         end
%     else
%         state=[state;0];
%     end
% if (value1<thr && value2<thr)||(value2<thr && value3<thr)||(value1<thr && value3<thr) 
    if (value1<thr &&  value2<thr && value3<thr )  
    whethertoescape=1;
    state=[state;1];
    limit = 100000;
    for k1 = 1 : 10 : limit
            %q_random=[ q_cool(end,1)+ratio(1)*step1*randi([-2,2]), q_cool(end,2)+ratio(1)*step1*randi([-2,2]), q_cool(end,3)+ratio(1)*step1*randi([-2,2]), q_cool(end,4)+ratio(1)*step1*randi([-2,2]), q_cool(end,5)+ratio(1)*step1*randi([-2,2]), 0];
            q_random=[ q_cool(end,1)+ratio(1)*k1*randi([-1,1]), q_cool(end,2)+ratio(1)*k1*randi([-1,1]), q_cool(end,3)+ratio(1)*k1*randi([-1,1]), q_cool(end,4)+ratio(1)*k1*randi([-1,1]), q_cool(end,5)+ratio(1)*k1*randi([-1,1]), 0];
            [bool] = check_collision(obstacle,q_random);
            if (q_random(1) > lowerLim(1) && q_random(1) < upperLim(1)) && (q_random(2) > lowerLim(2) && q_random(2) < upperLim(2))&& (q_random(3) > lowerLim(3) && q_random(3) < upperLim(3))&& (q_random(4) > lowerLim(4) && q_random(4) < upperLim(4))...
                    &&(q_random(5) > lowerLim(5) && q_random(5) < upperLim(5))&& (bool == 0) 
%              && norm(q_random - q_cool(end, :)) > k1
                q = q_random;
%                 whethertoescape=0;
                break
            elseif k1 == limit
                error('no viable q');
            end
    end
else 
    state=[state;0];
    end
end
end

