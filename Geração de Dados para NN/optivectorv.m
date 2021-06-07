function [optx,opty] = optivectorv(x,y)
    n = length(x);
    c=2;
    optx(1)=x(1);
    opty(1)=y(1);
    for i=2:n
       if ((x(i)-optx(c-1))>0.05 && (y(i) - opty(c-1))>0.05)
           optx(c)=x(i);
           opty(c)=y(i);
           c=c+1;
       end
    end
end