function [] = plot_solution(N,a,b,F_1,F_2) %plot the optimised variables
    delta_s = 1/N;  %step size
    s = 0:delta_s:1;
    s_1_2 = 0.5*delta_s:delta_s:1-delta_s*0.5;
    subplot(2,2,1);
    scatter(s_1_2,F_1,'.');
    ylabel('F_1(s)');xlabel('s');
    subplot(2,2,2);
    scatter(s_1_2,F_2,'.');     
    ylabel('F_2(s)');xlabel('s');
    subplot(2,2,3);
    scatter(s_1_2,a,'.')  
    ylabel('a(s)');xlabel('s');
    subplot(2,2,4);
    hold on
    scatter(s,b,'.');
    for i=1:length(s)-1 %draw linear spline
        m = (b(i+1)-b(i))/(s(i+1)-s(i));
        xspline = linspace(s(i),s(i+1),10);
        yspline = m*(xspline - s(i)) + b(i);
        plot(xspline,yspline,'-','color','k');
    end
    ylabel('b(s)');xlabel('s');
    %suptitle('Toy model: optimal circular spacial path tracking');
end

