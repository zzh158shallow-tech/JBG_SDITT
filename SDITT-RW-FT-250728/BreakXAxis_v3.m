

% Julie Haas, after Michael Robbins
% - assumes dx is same for all data
% - 'width' must be a whole number
% - to change axis, use {a=axis} to get axis, and reset in those units

%test data:  
% x=[1:.5:40]; y=rand(size(x)); start=10;stop=20;width=2; 
% h=BreakXAxis_v2(x,y,start,stop,width);
% 
% 
% x=.01:.01:10;y=sin(6*x);start=2;stop=3;width=0;
% h=BreakXAxis_v2(x,y,start,stop,width);

%%
function h=BreakXAxis_v3(x, y, start, stop, width, Num_Fig, Range_XTick, Range_XLim, LineWidth, i2, i2_max)
% x = (1 : 0.5 : 40);
% y = rand(size(x));
% start = 10;
% stop = 30;
% width = 1;

% x = Tar(:,1); y = Tar(:,2); start=12; stop=51; width=2;

% erase unused data
y(x>start & x<stop)=[];
x(x>start & x<stop)=[];

pos = find(x<=start, 1, 'last' );
x = [x(1:pos); NaN(2,1); x(pos+1:end)];
y = [y(1:pos); NaN(2,1); y(pos+1:end)];

% map to new xaxis, leaving a space 'width' wide
x2=x;
x2(x2>=stop)=x2(x2>=stop)-(stop-start-width);

if ~isempty(Range_XLim)
    Range_XLim(end) = Range_XLim(end)-(stop-start-width);
    xlim(Range_XLim)    % Added by Kayan Chan
end

figure(Num_Fig);
% if Choose_clf==1
%     clf                        % Added by Kayan Chan
% end
% h=plot(x2,y,'.');
h=plot(x2, y, 'Linewidth', LineWidth);
hold on               % Added by Kayan Chan
if ~isempty(Range_XTick)
    set(gca, 'XTick', Range_XTick)    % Added by Kayan Chan
end

if i2==i2_max
    Range_YTick=get(gca,'YLim');
    t1=text(start+width/2,min(Range_YTick),'//','fontsize',15);
    t2=text(start+width/2,max(Range_YTick),'//','fontsize',15);

%     ytick=get(gca,'YTick');
%     t1=text(start+width/2,ytick(1),'//','fontsize',15);
%     t2=text(start+width/2,ytick(max(length(ytick))),'//','fontsize',15);
    % For y-axis breaks, use set(t1,'rotation',270);

    % remap tick marks, and 'erase' them in the gap
    xtick=get(gca,'XTick');
    dtick=xtick(2)-xtick(1);
    gap=floor(width/dtick);
    last=max(xtick(xtick<=start));          % last tick mark in LH dataset
    next=min(xtick(xtick>=(last+dtick*(1+gap))));   % first tick mark within RH dataset
    offset=size(x2(x2>last&x2<next),2)*(x(2)-x(1));

    for i=1:sum(xtick>(last+gap))
        xtick(find(xtick==last)+i+gap)=stop+offset+dtick*(i-1);
    end

    for i=1:length(xtick)
        if xtick(i)>last&xtick(i)<next
            xticklabel{i}=sprintf('%d',[]);
        else
            xticklabel{i}=sprintf('%d',xtick(i));
        end
    end
    set(gca,'xticklabel',xticklabel);
end

