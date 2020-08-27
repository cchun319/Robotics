function grid = map_emp(map,grid)

boundary = map.bound_xyz;
x_min = boundary(1);
y_min = boundary(2);
z_min = boundary(3);
x_max = boundary(4);
y_max = boundary(5);
z_max = boundary(6);
margin = map.margin;  
blocks= map.blocks;

m=size(grid,1);
n=size(grid,2);
p=size(grid,3);

block_size= size(blocks,1);

grid=zeros(m,n,p);
for i =1 : m
    for j=1:n
        for k=1:p
            position= sub2pos(map,[i,j,k]);
                for j=1: block_size
                    for l=1: block_size
                        if (position(1) >=(blocks(l,1)-margin)) && (position(2) >=(blocks(l,2)-margin)) && (position(3) >=(blocks(l,3)-margin))&&...
                                (position(1)<=(blocks(l,4)+margin)) && (position(2) <=(blocks(l,5)+margin)) && (position(3)<=(blocks(l,6)+margin))
                            grid(i,j,k)=1;
                            break
                        end
                    end
                end
            end
        end
    end
end






