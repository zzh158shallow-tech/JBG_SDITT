%% 任意四边形等参元形函数，

% input_Con_Target_temp: Nz, Ny, Tx, Ty, Tz

function output_Con_Around  = Shape_Isoparametric_P(Coor_Target_local_XOY, Coor_Target_local_YOZ, input_Con_Target_temp)

output_Con_Around = zeros(4,5);

%%% XOY
zeta_local = Coor_Target_local_XOY(1);
eta_local  = Coor_Target_local_XOY(2);

N = [1/4*(1-zeta_local)*(1-eta_local) ...
	 1/4*(1-zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1-eta_local)];

output_Con_Around(:,2) = input_Con_Target_temp(1,2)*N;
output_Con_Around(:,3) = input_Con_Target_temp(1,3)*N;
output_Con_Around(:,4) = input_Con_Target_temp(1,4)*N;

%%% YOZ
zeta_local = Coor_Target_local_YOZ(1);
eta_local = Coor_Target_local_YOZ(2);

N = [1/4*(1-zeta_local)*(1-eta_local) ...
	 1/4*(1-zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1+eta_local) ...
	 1/4*(1+zeta_local)*(1-eta_local)];

output_Con_Around(:,1) = input_Con_Target_temp(1,1)*N;
output_Con_Around(:,5) = input_Con_Target_temp(1,5)*N;
