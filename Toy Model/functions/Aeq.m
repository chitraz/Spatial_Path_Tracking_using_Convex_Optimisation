function [Aeq] = Aeq(N,angle,phase,radius,mass)
    delta_s=1/N; 
    B_11 = (-2*delta_s)*eye(N);
    B_12 = [-1*eye(N) zeros(N,1)]+[zeros(N,1) eye(N)];
    B_13 = zeros(N);
    B_14 = zeros(N);
    B_21 = zeros(2,N);
    B_22 = zeros(2,N+1); B_22(1,1) = 1; B_22(2,N+1)=1; 
    B_23 = zeros(2,N);
    B_24 = zeros(2,N);
    [B_31,B_32,B_41,B_42] = M_C_k(N,angle,phase,radius,mass);
    B_33 = -1*eye(N);
    B_34 = zeros(N);
    B_43 = zeros(N);
    B_44 = -1*eye(N);

    Aeq = [B_11 B_12 B_13 B_14; B_21 B_22 B_23 B_24;...
        B_31 B_32 B_33 B_34; B_41 B_42 B_43 B_44 ];
end

