function [ B_31,B_32,B_41,B_42 ] = M_C_k(N,angle,phase,radius,mass)
    beta = angle; phi = phase; R = radius; m = mass;
    beta_sq = (beta)^2;
    delta_s = 1/N;
    s_k = delta_s*0.5;
    m_1 = zeros(1,N);
    m_2 = zeros(1,N); 
    c_1 = zeros(1,N);
    c_2 = zeros(1,N);
    temp = 0;
    for i=0:1:(N-1) 
     m_1(i+1) = -m*R*beta*sin(beta*s_k+phi);
     m_2(i+1) = m*R*beta*cos(beta*s_k+phi);
     temp = atan2(-sin(beta*s_k+phi),cos(beta*s_k+phi));
     c_1(i+1) = m*R*beta_sq*(sin(temp-pi/2)-cos(beta*s_k+phi));
     c_2(i+1) = m*R*beta_sq*(cos(temp-pi/2)-sin(beta*s_k+phi));
     s_k = s_k+delta_s;     
    end
    B_31 = diag(m_1); %construct the matrix with computed diagonal values
    B_32 = [0.5*diag(c_1) zeros(N,1)] + [zeros(N,1) 0.5*diag(c_1)];
    B_41 = diag(m_2);
    B_42 = [0.5*diag(c_2) zeros(N,1)] + [zeros(N,1) 0.5*diag(c_2)];
end