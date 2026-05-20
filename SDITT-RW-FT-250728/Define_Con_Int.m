function Con_Int = Define_Con_Int(InpPar)

% global InpPar.Type_Side InpPar.Nw InpPar.N_ConPatch InpPar.Exp_WS InpPar.Exp_DummyRail InpPar.NM_FW

Con_Int = struct;
Con_Int.Zwy = [];
Con_Int.Zsd = [];
Con_Int.Zjsd = [];
Con_Int.Pxt = [];

for i1 = 1:1:InpPar.Nw
    for i2 = 1:1:InpPar.N_ConPatch
        Con_Int.Dis_Rail.([InpPar.Exp_WS{i1},'_',InpPar.Exp_DummyRail{i2}]) = [];
        Con_Int.Vel_Rail.([InpPar.Exp_WS{i1},'_',InpPar.Exp_DummyRail{i2}]) = [];
        Con_Int.Acc_Rail.([InpPar.Exp_WS{i1},'_',InpPar.Exp_DummyRail{i2}]) = [];
    end
end

Con_Int.WR_Geometry = [];
for i3 = 1:1:2
    Con_Int.Wheel_1_Ia.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Wheel_1_Ib.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Ver_Pen_postive_start.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Ver_Pen_postive_end.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Wheel_2.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Rail_1.(InpPar.Type_Side{i3}) = struct;
    Con_Int.NF.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Prh.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Prhx.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Prhxf.(InpPar.Type_Side{i3}) = struct;
    Con_Int.RHXS.(InpPar.Type_Side{i3}) = struct;
    Con_Int.RHLv.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Vjd.(InpPar.Type_Side{i3}) = struct;
    Con_Int.Vsdc.(InpPar.Type_Side{i3}) = struct;
    Con_Int.RelVel.(InpPar.Type_Side{i3}) = struct;
    
    if InpPar.NM_FW>0
        Con_Int.ShapeFun_XOY.(InpPar.Type_Side{i3}) = struct;
        Con_Int.ShapeFun_Z.(InpPar.Type_Side{i3}) = struct;
        Con_Int.Pos_FW_global.(InpPar.Type_Side{i3}) = struct;
        Con_Int.Vel_FW_global.(InpPar.Type_Side{i3}) = struct;
    end
    
end

