function plot_bloos(Channel, tt, figure_num, Type_Line)

if ~exist('Type_Line')
    Type_Line = '-';
end

for i = 1:1:length(tt)
    bools = Channel{tt(i)}==0;
    xx = Channel{2};
    yy = Channel{tt(i)};
    xx(bools) = NaN;
    yy(bools) = NaN;    
    
    figure(figure_num)
    hold on
    plot(xx,yy,Type_Line);
    hold off
end


