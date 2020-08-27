function [ftotal,inside_obstacles]=potentialfield(p_start,p_final,obstacle)

for(j=1:5)
%attractive potential field and attractive force 

dividing_j = 5000;
k1=[1e5 0.1 0.1 0.2 0.2];

% map1
% dividing_j=10;
% k1=[0.1 0.1 0.1 0.2 0.2];

%map2
% dividing_j=10;
% k1=[0.1 0.1 0.1 0.2 0.2];

p1=p_start(j,:);
p2=p_final(j,:);
if (norm(p2-p1))<dividing_j
    f1(j,:)=-1*k1(j)*(p1-p2);
else
    f1(j,:)=-1*(p1-p2)/(norm(p2-p1));
end

% repulsive potential field and repulsive force 
% d0_j = 30;
% k2=[10 10 1e4 5e5 5e7];

% map1
d0_j = 30;
k2=[10 10 1e4 5e5 5e7];

% map2
% d0_j = 20;
% k2=[1e8 1e1 1e1 1e10 1e10];
number=size(obstacle,1);
for(i=1:number)    

[d(i),b(i,:)]=distPointToBox(p1,obstacle(i,:));
 
 if(d(i)<0)
    inside_obstacles=1;
    f2(j,:,i)=[1000,1000,1000];
 else
    inside_obstacles=0;
 end
 
  if(d(i) > d0_j)
      f2(j,:,i)=[0,0,0];
  else
        f2(j,:,i)=k2(j)*(1/(d(i))-1/(d0_j))*(1/(d(i)^2))*(p1-b(i,:))/(d(i));  
  end
end

f3(j,:)=[0 0 0];
   for(k=1:number)
       f3(j,:)=f3(j,:)+f2(j,:,k);
   end
% total force 
      ftotal(j,:)=f1(j,:)+f3(j,:);
end

end

