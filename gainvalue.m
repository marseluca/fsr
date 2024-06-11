function k = gainvalue(r,c)

x = floor(length(c)/2);

for i = 1:x
    temp = c(i);
    c(i) = c(end-i+1);
    c(end-i+1) = temp;
end

k = ones(r,1);
k(r) = c(end);

    for i=1:r-1
        prod = k(end);
        for j=1:i
            prod = prod*k(end-j);
        end
        k(r-i) = c(end-i)/prod;

    end
end