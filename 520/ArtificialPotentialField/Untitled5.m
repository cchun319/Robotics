p0=100;
k2=10;
d1=10000000000000;
number=size(obstacle,1);
for(i=1:number)    
d(i)=distPointToBox(p1,obstacle(i,:));
  if(d(i)<d1)
   d1=d(i);
  end
if(d(i)<0)
    inside_obstacles=1;
    break;
  else
  inside_obstacles=0;
  end
end