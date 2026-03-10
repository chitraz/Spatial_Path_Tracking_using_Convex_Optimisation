function [ B_31,B_32,B_41,B_42 ] = m_c(m1,m2,L1,L2,gamma,phi,alpha,N)
    delta_s = 1/N;
    s_k = delta_s*0.5;
    m_1 = zeros(1,N);
    m_2 = zeros(1,N); 
    c_1 = zeros(1,N);
    c_2 = zeros(1,N);
    for i=0:1:(N-1) 
     m_1(i+1) = (m1*L1^2+m2*(L1^2+L2^2+L1*L2*cos(phi*s_k+alpha)))*gamma...
         +(m2*(L2^2+L1*L2*cos(phi*s_k+alpha)))*phi; 
     m_2(i+1) = (m2*(L2^2+2*L1*L2*cos(phi*s_k+alpha)))*gamma + (m2*L2^2)*phi;
     c_1(i+1) = (phi*(-2*m2*L1*L2*sin(phi*s_k+alpha)))*gamma +...
         (phi*(-m2*L1*L2*sin(phi*s_k+alpha)))*phi;
     c_2(i+1) = (gamma*(m2*L1*L2*sin(phi*s_k+alpha)))*gamma;
     s_k = s_k+delta_s;     
    end
    B_31 = diag(m_1); %construct the matrix with computed diagonal values
    B_32 = [0.5*diag(c_1) zeros(N,1)] + [zeros(N,1) 0.5*diag(c_1)];
    B_41 = diag(m_2);
    B_42 = [0.5*diag(c_2) zeros(N,1)] + [zeros(N,1) 0.5*diag(c_2)];
end

