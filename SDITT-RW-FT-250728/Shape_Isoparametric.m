%% 任意四边形等参元形函数，

function [output_Con_Target]  = Shape_Isoparametric(Dir, Coor_Target_local, input_Con_Around_temp)

% num = 1;  % X, Y, Z
% Coor_Target_local = Coor_Target_local_XOY;  % zeta,eta
% input_Con_Around_temp = vel_Con_Around_Deformed_track{i,1}; % (4*4)

if strcmp(Dir, 'X')
    num = 2;
elseif strcmp(Dir, 'Y');
    num = 3;
elseif strcmp(Dir, 'Z')
    num = 4;
end

zeta_local = Coor_Target_local(1);
eta_local = Coor_Target_local(2);

N = [1/4*(1-zeta_local)*(1-eta_local) ...
	 1/4*(1-zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1-eta_local)];

% if ~strcmp(Opt, 'Load')
    output_Con_Target = N*input_Con_Around_temp(:,num);
%     output_Con_Around = [];
% else
%     output_Con_Target = [];
%     output_Con_Around = [input_Con_Target_temp(1)*N' ...
%                          input_Con_Target_temp(2)*N' ...
%                          input_Con_Target_temp(3)*N' ...
%                          input_Con_Target_temp(4)*N' ...
%                          input_Con_Target_temp(5)*N'];
% end

