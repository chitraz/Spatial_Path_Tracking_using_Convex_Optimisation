function [a,b,tau_1,tau_2] = ConvexProgram_elbow(m_1,m_2,L_1,L_2,gamma,beta,phi,alpha,f_s1,f_s2,N,g,b_max,tau1_max,...
    tau1_min,tau2_max,tau2_min,b_0, b_N,theta1_ddot_min,theta1_ddot_max,theta2_ddot_min,theta2_ddot_max)

    delta_s = 1/N; %step size
    s_k = delta_s*0.5;
    Aeq_ =  Aequality(m_1,m_2,L_1,L_2,gamma,phi,alpha,N);
    beq = bequality(m_1,m_2,L_1,L_2,gamma,beta,phi,alpha,N,f_s1,f_s2,g,b_0,b_N);   
    tau_lb = [tau1_min*ones(N,1);tau2_min*ones(N,1)];       
    tau_ub = [tau1_max*ones(N,1);tau2_max*ones(N,1)];
    b_ub = b_max*ones(N+1,1);
    b_lb = zeros(N+1,1);
    a_ub_1 = (theta1_ddot_max/gamma)*ones(N,1);    
    a_lb_1 = (theta1_ddot_min/gamma)*ones(N,1); 
    a_ub_2 = (theta2_ddot_max/phi)*ones(N,1); 
    a_lb_2 = (theta2_ddot_min/phi)*ones(N,1); 
    
    cvx_solver 
    cvx_begin
    variable X(4*N+1);
    for i=0:1:N-1
        time_opt_term(i+1) = 2*delta_s/(sqrt(X(N+1+i))+sqrt(X(N+2+i)));
    end
    time_opt_term = sum(time_opt_term)
    for i=1:1:N
        energy_opt_term(i+1) = delta_s*abs(X(2*N+1+i)*gamma + X(3*N+1+i)*phi);
        s_k = s_k+delta_s;  
    end
    energy_opt_term = sum(energy_opt_term)
    minimize(energy_opt_term)
    %minimize(0) %fesibility problem
    subject to
        Aeq_*X == beq; %equality constriants
        X(2*N+2:4*N+1) >= tau_lb;
        X(2*N+2:4*N+1) <= tau_ub; %tau bounds
        X(N+1:2*N+1) >= b_lb;
        X(N+1:2*N+1)<= b_ub; %b bound
        X(1:N) <= a_ub_1; 
        X(1:N) <= a_ub_2;   %a bounds
        X(1:N) >= a_lb_1;
        X(1:N) >= a_lb_2;
    cvx_end

    a = X(1:N)';
    b = X(N+1:2*N+1)';
    tau_1 = X(2*N+2:3*N+1)';  %extration
    tau_2 = X(3*N+2:4*N+1)';
end

