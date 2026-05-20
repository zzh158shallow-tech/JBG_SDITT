%% 结果输出函数 I 

function [ZP_Dyn, ZP_Con] = Output_ZP_FW_220713(InpPar, ZP_Dyn, ZP_Con, xlcs, Par_Track, Con_WS, Zwy, Pjcc, Pjch, Prhxf, Dis_Rail, Vel_Rail, Acc_Rail, RailPro_ProCS, drtaT, i_drtaT, times_drtaT, T, DamperNL)

% global InpPar.Exp_WS InpPar.Exp_DummyRail
% global InpPar.Nw InpPar.NM_FW InpPar.N_track InpPar.N_ConPatch InpPar.Type_Side InpPar.Type_Normal
% global ZP_Dyn ZP_Con xlcs Par_Track

Ori_prr = Par_Track.Ori_prr;

%% ====== ZP_Dyn - 导向轮对
i1 = 1;
Con_str = Con_WS.(InpPar.Exp_WS{i1});
ZP_Dyn.T(1+xlcs,1) = T;
ZP_Dyn.drtaT(1+xlcs,:) = [Con_str.Mileage, drtaT, i_drtaT, times_drtaT];
ZP_Dyn.Yw(1+xlcs,:) = [Con_str.Mileage, Zwy(InpPar.N_track+InpPar.NM_FW*InpPar.Nw+5*(i1-1)+2,4)];
ZP_Dyn.Vzd(1+xlcs,:)     = [Con_str.Mileage, Con_str.Vzdli, Con_str.Vzdlj, Con_str.Vzdlk];
ZP_Dyn.Mileage(1+xlcs,1:2) = [Con_str.Mileage, RailPro_ProCS.L1.FF_Profile_num];
for k = 2:1:length(InpPar.Exp_DummyRail)
    Tk = InpPar.Exp_DummyRail{k};
    if isempty(RailPro_ProCS.(Tk).FF_Profile_num)
        ZP_Dyn.Mileage(1+xlcs,k+1) = NaN;
    else
        ZP_Dyn.Mileage(1+xlcs,k+1) = RailPro_ProCS.(Tk).FF_Profile_num;
    end
end
ZP_Dyn.Pjcc(1+xlcs,:) = [Con_str.Mileage, Pjcc'];
ZP_Dyn.Pjch(1+xlcs,:) = [Con_str.Mileage, Pjch'];

% DamperNL
if ~isempty(fieldnames(DamperNL))
    ZP_Dyn.DamperNL.DPz.DampVel(1+xlcs,:) = [Con_str.Mileage, DamperNL.DPz.DampVel'];
    ZP_Dyn.DamperNL.DPz.SpringLength(1+xlcs,:) = [Con_str.Mileage, DamperNL.DPz.SpringLength'];
    
    ZP_Dyn.DamperNL.Sx.DampVel(1+xlcs,:) = [Con_str.Mileage, DamperNL.Sx.DampVel'];
    ZP_Dyn.DamperNL.Sx.SpringLength(1+xlcs,:) = [Con_str.Mileage, DamperNL.Sx.SpringLength'];
    
    ZP_Dyn.DamperNL.DPy.DampVel(1+xlcs,:) = [Con_str.Mileage, DamperNL.DPy.DampVel'];
    ZP_Dyn.DamperNL.DPy.SpringLength(1+xlcs,:) = [Con_str.Mileage, DamperNL.DPy.SpringLength'];
    
    ZP_Dyn.DamperNL.STy.SpringLength(1+xlcs,:) = [Con_str.Mileage, DamperNL.STy.SpringLength'];
    ZP_Dyn.DamperNL.STy.Force(1+xlcs,:) = [Con_str.Mileage, DamperNL.FY_STy_Stiff'];    
end

%% ====== ZP_Dyn - 所有轮对
for i1 = 1:1:InpPar.Nw    
    Con_str = Con_WS.(InpPar.Exp_WS{i1});
    
    ZP_Dyn.WR_Geometry_FW{i1,1}(1+xlcs, :) = [Con_str.Mileage, Con_str.WR_Geometry];
    for i2 = 1:1:length(InpPar.Type_Side)
        T2 = InpPar.Type_Side{i2};
        for k = 1:1:size(Con_str.Normal_Force.(T2),1)
            ZP_Dyn.Con_Rail_1.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_rail_1.(T2)(k,:)];
            ZP_Dyn.Con_Rail_2.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_rail_2.(T2)(k,:)];
            ZP_Dyn.Con_Wheel_1.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_1.(T2)(k,:)];
            ZP_Dyn.Con_Wheel_2.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_2.(T2)(k,:) Con_str.elastic_permeability_Unit.(T2)(k)];
            ZP_Dyn.Normal_Force.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Normal_Force.(T2)(k,:)];
            ZP_Dyn.Prhx.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Prhx_T.(T2)(k,:)];
            ZP_Dyn.Prhxf.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.Prhxf_T.(T2)(k,:)];
            ZP_Dyn.a2.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.a2.(T2)(k,:)];
            ZP_Dyn.b2.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.b2.(T2)(k,:)];
            ZP_Dyn.RHXS.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.RHXS.(T2)(k,:)];
            ZP_Dyn.RHLv.(T2){i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.RHLv.(T2)(k,:)];
            if InpPar.NM_FW>0
                ZP_Dyn.ShapeFun_FW.(T2).XOY{i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.ShapeFun_FW.(T2).XOY{k,1}];
                ZP_Dyn.ShapeFun_FW.(T2).Z{i1,k}(1+xlcs,:) = [Con_str.Mileage, Con_str.ShapeFun_FW.(T2).Z{k,1}];
                ZP_Dyn.DOF_pos_FW.(T2).Node_Around_XOY{i1,k}(1+xlcs,:) = [Con_str.Mileage, reshape(Con_str.DOF_pos_FW.(T2).Node_Around_XOY{k,1},12,1)'];
                ZP_Dyn.DOF_pos_FW.(T2).Node_Around_Z{i1,k}(1+xlcs,:) = [Con_str.Mileage, reshape(Con_str.DOF_pos_FW.(T2).Node_Around_Z{k,1},6,1)'];
%                 ZP_Dyn.Wheel_2_Rigid.(T2)(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_2_Rigid.(T2)(k,:)];
            end
        end        
    end
    
    for i_Patch = 1:1:InpPar.N_ConPatch
        tt.(InpPar.Exp_DummyRail{i_Patch}) = 1;
    end
    for i2 = 1:1:length(InpPar.Type_Side)
        T2 = InpPar.Type_Side{i2};
        for k = 1:1:size(Con_str.Normal_Force.(T2),1)
            i_Patch =  Con_str.Normal_Force.(T2)(k,4);
            T3 = InpPar.Exp_DummyRail{i_Patch};
            ZP_Con.RelVel.(T3){i1,tt.(T3)}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_RelVel.(T2)(k,:)];
            ZP_Con.RelVel_max.(T3){i1,tt.(T3)}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_RelVel_max.(T2)(k,:)];
            tt.(T3) = tt.(T3)+1;
        end
    end
    
end

%% ====== ZP_Dyn - 轨下位移、速度和加速度,纵向、横向、垂向轮轨力
for i1 = 1:1:InpPar.Nw
    T1 = InpPar.Exp_WS{i1};
    for i2 = 1:1:InpPar.N_ConPatch
        T2 = InpPar.Exp_DummyRail{i2};
        i_Patch = InpPar.N_ConPatch*(i1-1)+i2;
        ZP_Dyn.Dis_Rail.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, Dis_Rail(i_Patch,:)];
        ZP_Dyn.Vel_Rail.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, Vel_Rail(i_Patch,:)];
        ZP_Dyn.Acc_Rail.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, Acc_Rail(i_Patch,:)];
        ZP_Dyn.FX.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, -Prhxf(i_Patch,1)];
        ZP_Dyn.FY.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, -Prhxf(i_Patch,2)-Pjch(i_Patch,1)];
        ZP_Dyn.FZ.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, -Prhxf(i_Patch,3)-Pjcc(i_Patch,1)];
        ZP_Dyn.Dis_TIrr.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, Con_WS.(T1).Dis_TIrr{1,i2}];
        ZP_Dyn.Vel_TIrr.(T1).(T2)(1+xlcs,:) = [Con_WS.(T1).Mileage, Con_WS.(T1).Vel_TIrr{1,i2}];
    end
    ZP_Dyn.VF(1+xlcs,i1*2-1:i1*2) = [ZP_Dyn.FZ.(T1).L1(1+xlcs,2), ZP_Dyn.FZ.(T1).R1(1+xlcs,2)+ZP_Dyn.FZ.(T1).R2(1+xlcs,2)+ZP_Dyn.FZ.(T1).R3(1+xlcs,2)];
    ZP_Dyn.LF(1+xlcs,i1*2-1:i1*2) = [ZP_Dyn.FY.(T1).L1(1+xlcs,2), ZP_Dyn.FY.(T1).R1(1+xlcs,2)+ZP_Dyn.FY.(T1).R2(1+xlcs,2)+ZP_Dyn.FY.(T1).R3(1+xlcs,2)];
end

%% ZP_Con - 接触信息
for i1 = 1:1:InpPar.Nw
    Con_str = Con_WS.(InpPar.Exp_WS{i1});
    
    for i_Patch = 1:1:InpPar.N_ConPatch
        tt.(InpPar.Exp_DummyRail{i_Patch}) = 1;
    end
    
    for i2 = 1:1:length(InpPar.Type_Side)
        T2 = InpPar.Type_Side{i2};
        for k = 1:1:size(Con_str.Normal_Force.(T2),1)
            i_Patch =  Con_str.Normal_Force.(T2)(k,4);
            Tk = InpPar.Exp_DummyRail{i_Patch};
            dY = Con_str.profile_r_dY.(Tk);
            dZ = Con_str.profile_r_dZ.(Tk);
            Target_tt = tt.(Tk);
            ZP_Con.Rail_1.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_rail_1.(T2)(k,:)];
            ZP_Con.Rail_2.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_rail_2.(T2)(k,:)];
%             ZP_Con.Rail_2.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, (Con_str.Con_rail_1.(T2)(k,1)-(0.7175+Ori_prr)*(-1+2*sign(i2-1))-dY)*1e3, (Con_str.Con_rail_1.(T2)(k,2)-dZ)*1e3];
            ZP_Con.Wheel_2.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_2.(T2)(k,1:end-1), Con_str.elastic_permeability_Unit.(T2)(k)];
            ZP_Con.Area.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, (pi*Con_str.a2.(T2)(k,:)*Con_str.b2.(T2)(k,:))*1e6];
            ZP_Con.Area_STRIPES.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Area_STRIPES.(T2)(k,:)*1e6];            
            ZP_Con.Angle.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_2.(T2)(k,6)];
            ZP_Con.CX.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.RHLv.(T2)(k,1)];
            ZP_Con.CY.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.RHLv.(T2)(k,2)];
            ZP_Con.CSpin.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.RHLv.(T2)(k,3)];
            ZP_Con.CFX.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Prhx_T.(T2)(k,1)];
            ZP_Con.CFY.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Prhx_T.(T2)(k,2)];
            ZP_Con.CFSpin.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Prhx_T.(T2)(k,3)];
            ZP_Con.NF.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Normal_Force.(T2)(k,:)];
            ZP_Con.Vjd.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Vjd.(T2)(k,:)];
            ZP_Con.Vjd_r.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Vjd_r.(T2)(k,:)];
            ZP_Con.Vsdc.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Vsdc.(T2)(k,:)];
            ZP_Con.Vjsdc.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Vjsdc.(T2)(k,:)];
            ZP_Con.Vgd.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Vgd.(T2)(k,:)];
            ZP_Con.R_yy_w.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.R_yy_w.(T2)(k,1)];
            ZP_Con.R_xx_w.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.R_xx_w.(T2)(k,1)];
            ZP_Con.R_xx_r.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.R_xx_r.(T2)(k,1)];
            ZP_Con.rou.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.rou.(T2)(k,1)];
            
            if InpPar.NM_FW>0
                ZP_Con.Wheel_2_Rigid.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_wheel_2_Rigid.(T2)(k,1:end-1), Con_str.elastic_permeability_Unit.(T2)(k)];
%                 ZP_Con.Rail_2_Rigid.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Con_rail_2_Rigid.(T2)(k,:)];
            end

            if strcmp(InpPar.Type_Normal, 'STRIPES&ConDamp')
                if ~isempty(Con_str.Epsilon.(T2))
                    ZP_Con.Epsilon.(Tk){i1,Target_tt}(1+xlcs,:) = [Con_str.Mileage, Con_str.Epsilon.(T2)(k)];
                    ZP_Con.STRIPES_Stiff_Pen.(Tk){i1,Target_tt}{1+xlcs,1} = Con_str.Mileage;
                    ZP_Con.STRIPES_Stiff_Pen.(Tk){i1,Target_tt}{1+xlcs,2} = Con_str.Con_STRIPES.(T2){k,4};
                    ZP_Con.STRIPES_Stiff_Pen.(Tk){i1,Target_tt}{1+xlcs,3} = Con_str.Con_STRIPES.(T2){k,2};
                end
            end
            
            tt.(Tk) = tt.(Tk)+1;
        end
    end
    
end

