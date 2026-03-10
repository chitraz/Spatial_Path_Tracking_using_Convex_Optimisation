function [a,b,F_1,F_2] = ConvexProgram(N,g,angle,phase,radius,mass,f_max,f_min,accel_max,accel_min,b_max,b_0,b_N,sigma,gamma,omega,f_s1,f_s2)
    delta_s = 1/N; %step size
    s_k = delta_s*0.5;
    f_T_ = f_T(N,angle,phase,radius);    %construct objective
    A_ = A(N,angle,phase,radius,mass);
    b = [accel_max*ones(1,2*N) -accel_min*ones(1,2*N)]; b = b';
    Aeq_ = Aeq(N,angle,phase,radius,mass);   
    beq = Beq(N,angle,phase,mass,b_0,b_N,g,f_s1,f_s2); beq = beq';            
    f_lb = f_min*ones(2*N,1);           %construct constraints
    f_ub = f_max*ones(2*N,1);
    b_ub = b_max*ones(N+1,1);
    b_lb = zeros(N+1,1);
    
    cvx_solver %cvx: my convex program -> standard cone program -> applys standard solver (IPM based)
    cvx_begin
    variable X(4*N+1);
    work_opt_term = f_T_*X
    for i=0:1:N-1
        time_opt_term(i+1) = 2*delta_s/(sqrt(X(N+1+i))+sqrt(X(N+2+i)));
    end
    time_opt_term = sum(time_opt_term)
    for i=1:1:N
        energy_opt_term(i+1) = delta_s*abs(X(2*N+1+i)*angle*radius*cos(angle*s_k+phase)...
            -X(3*N+1+i)*angle*radius*sin(angle*s_k+phase));
        s_k = s_k+delta_s;  
    end
    energy_opt_term = sum(energy_opt_term)
    minimize(omega*energy_opt_term + sigma*work_opt_term + gamma*time_opt_term); % affine term combined with a convex term in an affine way, so convex
    %minimize(0) %fesibility problem
    subject to
        Aeq_*X == beq;
        A_*X <= b;
        f_lb <= X(2*N+2:4*N+1);
        X(2*N+2:4*N+1) <= f_ub; %F bounds
        b_lb <= X(N+1:2*N+1);
        X(N+1:2*N+1)<= b_ub; %b bound
    cvx_end

    a = X(1:N)';
    b = X(N+1:2*N+1)';
    F_1 = X(2*N+2:3*N+1)';  %extration
    F_2 = X(3*N+2:4*N+1)';
end

