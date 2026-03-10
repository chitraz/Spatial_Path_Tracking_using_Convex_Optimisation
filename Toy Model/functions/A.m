 function [A] = A(N,beta,phi,R,m)
    beta_sq = (beta)^2;
    delta_s = 1/N;
    s_k = delta_s*0.5;
    x1_dash = zeros(1,N); x2_dash = zeros(1,N);
    x1_ddash = zeros(1,N); x2_ddash = zeros(1,N);
    for i=0:1:(N-1) 
    x1_dash(i+1) = -R*beta*sin(beta*s_k + phi);
    x2_dash(i+1) = R*beta*cos(beta*s_k + phi);
    x1_ddash(i+1) = -R*beta_sq*cos(beta*s_k + phi);
    x2_ddash(i+1) = -R*beta_sq*sin(beta*s_k + phi);
    s_k = s_k+delta_s;    
    end
    A = [diag(x1_dash) diag(x1_ddash) zeros(N,1) zeros(N,N) zeros(N,N);...
        diag(x2_dash) diag(x2_ddash) zeros(N,1) zeros(N,N) zeros(N,N);...
        -diag(x1_dash) -diag(x1_ddash) zeros(N,1) zeros(N,N) zeros(N,N);...
        -diag(x2_dash) -diag(x2_ddash) zeros(N,1) zeros(N,N) zeros(N,N)];
end

