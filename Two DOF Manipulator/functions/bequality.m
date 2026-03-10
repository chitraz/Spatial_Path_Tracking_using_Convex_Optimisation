function [ beq ] = bequality(m1,m2,L1,L2,gamma,beta,phi,alpha,N,f_s1,f_s2,g,b_0,b_N)
    delta_s = 1/N;
    s_k = delta_s*0.5;
    beq = [zeros(1,N) b_0 b_N zeros(1,N) zeros(1,N)];
    for i=1:1:N 
      beq(N+2+i) = g*L1*(m1+m2)*cos(gamma*s_k+beta)+...
          m2*g*L2*cos((gamma+phi)*s_k+(beta+alpha)) - f_s1*sign(gamma);
      beq(2*N+2+i) = m2*g*L2*cos((gamma+phi)*s_k+(beta+alpha))...
           - f_s2*sign(phi);
      s_k = s_k+delta_s;
    end
    beq = beq';
end

