function [ftotal,inside_obstacles]=randi_potentialfield(p,p_final,obstacle,boundary)

x_length = double(boundary(4) - boundary(1));
y_length = double(boundary(5) - boundary(2));
z_length = double(boundary(6) - boundary(3));

x_min = boundary(1)+0.5*x_length*rand(1);
y_min = boundary(2)+0.5*y_length*rand(1);
z_min = boundary(3)+0.5*z_length*rand(1);
x_max = boundary(4)-0.5*x_length*rand(1);
y_max = boundary(5)-0.5*y_length*rand(1);
z_max = boundary(6)-0.5*z_length*rand(1);
% while 1
%     r_rand = [7*rand(1),7*rand(1),7*rand(1)]
%     if norm(r_rand)> 3
%         randiobstacle(1,1:3)=r_rand+p(5,:);
%         randiobstacle(1,4:6)=randiobstacle(1,1:3)+2;
%         break;
%     end
% end

% x_min = p(1)+5;
% y_min = p(2)+5;
% z_min = p(3)+5;
% x_max = x_min+0.5;
% y_max = y_min+0.5;
% z_max = z_min+0.5;
% % 
% randimin=2*p-p_final;
% randimax=randimin+1;
randiobstacle=[x_min,y_min,z_min,x_max,y_max,z_max];
for(j=1:5)
%attractive potential field and attractive force 
dividing_j=10;
k1_j=0.1;
p1=p(j,:);
p2=p_final(j,:);
if (norm(p2-p1))<dividing_j
    f1(j,:)=-1*k1_j*(p1-p2);
else
    f1(j,:)=-1*(p1-p2)/(norm(p2-p1));
end

% repulsive potential field and repulsive force 
d0_j=10;
k2_j=10;
d1=10000000000000;
% 

obstacle=[obstacle;randiobstacle];
number=size(obstacle,1);

for(i=1:number)    

[d(i),b(i,:)]=distPointToBox(p1,obstacle(i,:));
    % find shortest distance 
    if(d(i)<d1)
      d1=d(i);
      b(1,:)=b(i,:);
    end
    
 if(d(i)<0)
    inside_obstacles=1;
    f2(j,:)=[1000,1000,1000];
 else
    inside_obstacles=0;
  end
end
  if(d1>d0_j)
      f2(j,:)=[0,0,0];
  else
      f2(j,:)=k2_j*(1/(d1)-1/(d0_j))*(1/(d1*d1))*(p1-b(1,:))/d1;
  end
  

% total force 
  ftotal(j,:)=f1(j,:)-f2(j,:);
    end
end


