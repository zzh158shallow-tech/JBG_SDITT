function plot_cell(Cell_1,Cell_2,Cell_3,col_x,col_y,cof,Type_simulation)

% Cell_1 = ZP_FZ_Stock_L;
% Cell_2 = ZP_FZ_Stock_R;
% Cell_3 = ZP_FZ_Switch_R;
% col_x = 1;
% col_y = 2;
% Cell_1 = ZP_Con_Rail_Stock_L;
% Cell_2 = {};
% Cell_3 = {};

if isempty(Cell_3)
    sum = 2;
elseif isempty(Cell_2)   
    sum = 1;
else
    sum = 3;
end

for k = 1:1:sum
    expression = ['temp = Cell_',num2str(k),';'];
    eval(expression);
    for kk = 1:1:size(temp,2)
        bools = (temp{1,kk}(:,col_y)==0);
        temp{1,kk}(bools,:) = NaN;
        
        hold on
        if strcmp(Type_simulation, 'Cal')
            plot(temp{1,kk}(:,col_x),temp{1,kk}(:,col_y)*cof);
        elseif strcmp(Type_simulation, 'Preload')
            plot(1:1:length(temp{1,kk}(4:end,1)),temp{1,kk}(4:end,col_y)*cof);
        end
        hold off
    end
end
