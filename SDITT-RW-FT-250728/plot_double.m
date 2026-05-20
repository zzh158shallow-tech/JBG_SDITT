function plot_double(Matrix_1,Matrix_2,Matrix_3,col_x,col_y,cof,Type_simulation,Type_line)

% Matrix_1 = ZP_FZ_Stock_L;
% Matrix_2 = ZP_FZ_Stock_R;
% Matrix_3 = [];
% col_x = 1;
% col_y = 2;

if isempty(Matrix_3)
    sum = 2;
elseif isempty(Matrix_2)   
    sum = 1;
else
    sum = 3;
end

if ~exist('Type_line')
    Type_line = '-';
end

for k = 1:1:sum
    expression = ['temp = Matrix_',num2str(k),';'];
    eval(expression);
%     bools = (temp(:,col_y)==0);
%     temp(bools,:) = NaN;
    
%     figure(figure_num)
    hold on
    if strcmp(Type_simulation, 'Cal')
        plot(temp(4:end,col_x),temp(4:end,col_y)*cof, Type_line);
    elseif strcmp(Type_simulation, 'Preload')
        plot(1:1:length(temp(4:end,1)),temp(4:end,col_y)*cof, Type_line);
    end
end