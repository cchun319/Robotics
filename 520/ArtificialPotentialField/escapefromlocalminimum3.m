function [whethertoescape,currentstate]=escapefromlocalminimum3(q_cool,thr)


number = size(q_cool,1);

whethertoescape=0;
q=[0 0 0 0 0 0];

j = number - 3;
value1=norm(q_cool(j+1,:)-q_cool(j,:));
value2=norm(q_cool(j+2,:)-q_cool(j,:));
value3=norm(q_cool(j+3,:)-q_cool(j,:));
    
    if (value1 < thr &&  value2 < thr && value3 < thr )
        whethertoescape=1;
        currentstate=1;
    else
        whethertoescape=0;
        currentstate=0;
    end
end

