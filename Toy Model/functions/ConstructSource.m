function [F_1_t,F_2_t] = ConstructSource(N,t_s,F_1,F_2) %create a input source(time domain) for the simulink model
    N_spl = 10; %points inbetween each spline
    F_1_yspline = zeros(N*N_spl); F_1_xspline = zeros(N*N_spl);
    F_2_yspline = zeros(N*N_spl); F_2_xspline = zeros(N*N_spl);
    for i=1:length(t_s)-1 %linear spline
        m1 = (F_1(i+1)-F_1(i))/(t_s(i+1)-t_s(i));
        xspline1 = linspace(t_s(i),t_s(i+1),N_spl);
        yspline1 = m1*(xspline1 - t_s(i)) + F_1(i);
        m2 = (F_2(i+1)-F_2(i))/(t_s(i+1)-t_s(i));
        xspline2 = linspace(t_s(i),t_s(i+1),N_spl);
        yspline2 = m2*(xspline2 - t_s(i)) + F_2(i);
        if i==1
           F_1_yspline = yspline1; F_1_xspline = xspline1;
           F_2_yspline = yspline2; F_2_xspline = xspline2;
        else
           F_1_yspline = [F_1_yspline yspline1];
           F_1_xspline = [F_1_xspline xspline1];
           F_2_yspline = [F_2_yspline yspline2];
           F_2_xspline = [F_2_xspline xspline2];
        end
    end
    F_1_t = timeseries(F_1_yspline,F_1_xspline);
    F_2_t = timeseries(F_2_yspline,F_2_xspline);
end
