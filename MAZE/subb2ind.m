function idx = subb2ind(M, r, c, w)

idx = r + (c-1)*size(M,1) + (w-1)*size(M,1)*size(M,2); 

end