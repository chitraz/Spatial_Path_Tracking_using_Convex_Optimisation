function [ Beq ] = Beq(N,angle,phase,mass,b_0,b_N,g,f_s1,f_s2)
    beta = angle; phi = phase; m = mass;
    delta_s = 1/N;
    s_k = delta_s*0.5;
    Beq = [zeros(1,N) b_0 b_N zeros(1,N) zeros(1,N)];
    
    for i=1:1:N 
      Beq(N+2+i) = -g*m - f_s1*sign(-sin(beta*s_k + phi));   %sgn(x_1')
      Beq(2*N+2+i) =  - f_s2*sign(cos(beta*s_k + phi));     %sgn(x_2')
      s_k = s_k+delta_s;
    end

end