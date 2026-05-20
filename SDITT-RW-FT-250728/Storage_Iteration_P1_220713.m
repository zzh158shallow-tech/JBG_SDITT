%% 保存第 xcs 迭代分步的计算结果

function Con_Int = Storage_Iteration_P1_220713(InpPar, Con_Int, Con_str, i1, xcs, Mileage, DamperNL)

% global InpPar.Type_Side InpPar.NM_FW InpPar.Exp_WS

Con_Int.WR_Geometry.(InpPar.Exp_WS{i1})(xcs,:)= Con_str.WR_Geometry;
for i3 = 1:1:2
    T3 = InpPar.Type_Side{i3};
    len = size( Con_str.Normal_Force.(T3), 1);
    
    for k = 1:1:len
        Con_Int.Wheel_1_Ia.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Con_wheel_1_I.a.(T3)(k,:);
        Con_Int.Wheel_1_Ib.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Con_wheel_1_I.b.(T3)(k,:);
        
        if ~isempty( Con_str.Ver_Pen_postive_start.(T3) )
            Con_Int.Ver_Pen_postive_start.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Ver_Pen_postive_start.(T3)(k,:);
            Con_Int.Ver_Pen_postive_end.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Ver_Pen_postive_end.(T3)(k,:);
        end
        
        Con_Int.Wheel_2.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = [Mileage, Con_str.Con_wheel_2.(T3)(k,:), Con_str.elastic_permeability_Unit.(T3)(k)];
        Con_Int.Rail_1.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = [Mileage, Con_str.Con_rail_1.(T3)(k,:)];
        Con_Int.Rail_2.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = [Mileage, Con_str.Con_rail_2.(T3)(k,:)];
        Con_Int.NF.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Normal_Force.(T3)(k,:);
        Con_Int.Prh.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Prh.(T3)(k,:);
        Con_Int.Prhx_T.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Prhx_T.(T3)(k,:);
        Con_Int.Prhxf_T.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Prhxf_T.(T3)(k,:);
        Con_Int.RHXS.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.RHXS.(T3)(k,:);
        Con_Int.RHLv.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.RHLv.(T3)(k,:);
        Con_Int.Vjd.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Vjd.(T3)(k,:);
        Con_Int.Vjd_r.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Vjd_r.(T3)(k,:);
        Con_Int.Vsdc.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Vsdc.(T3)(k,:);
        Con_Int.RelVel.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.Con_RelVel.(T3)(k,:);
        
        if InpPar.NM_FW > 0
            Con_Int.ShapeFun_FW.(T3).XOY.(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.ShapeFun_FW.(T3).XOY{k,1};
            Con_Int.ShapeFun_FW.(T3).Z.(InpPar.Exp_WS{i1}){1,k}(xcs,:) = Con_str.ShapeFun_FW.(T3).Z{k,1};
            Con_Int.Pos_FW_global.(T3).(InpPar.Exp_WS{i1}){xcs,k} = Con_str.Pos_FW_global.(T3){k,1};
            Con_Int.Vel_FW_global.(T3).(InpPar.Exp_WS{i1}){xcs,k} = Con_str.Vel_FW_global.(T3){k,1};        
%             Con_Int.Wheel_2_Rigid.(T3).(InpPar.Exp_WS{i1}){1,k}(xcs,:) = [Mileage, Con_str.Con_wheel_2_Rigid.(T3)(k,:), Con_str.elastic_permeability_Unit.(T3)(k)];
        end
        
    end
end

Con_Int.DamperNL.DPz.Mark(xcs,:) = DamperNL.DPz.Mark';
Con_Int.DamperNL.Sx.Mark(xcs,:) = DamperNL.Sx.Mark';
Con_Int.DamperNL.DPy.Mark(xcs,:) = DamperNL.DPy.Mark';

