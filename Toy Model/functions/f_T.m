function [f_T] = f_T(N,angle,phase,radius)
    delta_s = 1/N;
    R = radius;
    beta = angle;
    phi = phase;
    s_k = delta_s*0.5;
    f_T = zeros(1,4*N+1); %preallocations
    for i=0:1:(N-1) %F_1
     f_T(2*N+2+i) = -beta*R*sin(beta*s_k+phi);
     s_k = s_k+delta_s;
    end
    s_k = delta_s*0.5;
    for i=0:1:(N-1) %F_2
     f_T(3*N+2+i) = beta*R*cos(beta*s_k+phi);
     s_k = s_k+delta_s;
    end
end

