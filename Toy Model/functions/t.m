function [t] = t(b, N) %get t-s relation
    sum = 0;
    delta_s = 1/N;
    t = zeros(1,N); %preallocation 
    for i=1:1:N  
       sum = sum + 2*delta_s/(sqrt(b(i))+sqrt(b(i+1)));
       t(i) = sum;
    end
end

