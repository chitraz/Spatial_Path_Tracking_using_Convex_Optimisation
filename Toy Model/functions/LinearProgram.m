function [a,b,F_1,F_2] = LinearProgram(N,g,angle,phase,radius,mass,f_max,f_min,accel_max,accel_min,b_max,b_0,b_N,options,f_s1,f_s2)
    f_T_ = f_T(N,angle,phase,radius);    %construct objective
    A_ = A(N,angle,phase,radius,mass);
    b = [accel_max*ones(1,2*N) -accel_min*ones(1,2*N)]; b = b';
    Aeq_ = Aeq(N,angle,phase,radius,mass,mu);   
    beq = Beq(N,angle,phase,mass,b_0,b_N,g,f_s1,f_s2); beq = beq';             
    f_lb = f_min*ones(2*N,1);           %construct constrains
    f_ub = f_max*ones(2*N,1);
    b_ub = b_max*ones(N+1,1);
    b_lb = zeros(N+1,1);
    UB = [inf*ones(N,1); b_ub; f_ub];
    LB = [-inf*ones(N,1); b_lb; f_lb];

    [X,E,exitflag,output,lambda]=linprog(f_T_,A_,b,Aeq_,beq,LB,UB,[],options);
    Algorithm_used = output.algorithm 
    No_of_iterations_taken = output.iterations  %display more info
    Solver_outcome = output.message 
    a = X(1:N)';
    b = X(N+1:2*N+1)';
    F_1 = X(2*N+2:3*N+1)';  %extration
    F_2 = X(3*N+2:4*N+1)';
end

