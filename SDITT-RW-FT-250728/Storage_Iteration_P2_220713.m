 %% 存储迭代不平衡项
 %%% ZP_Zwy_int：积分步、导向轮对里程、drtaT、位移、速度、加速度、荷载、法向力迭代误差、法向力切向力合力迭代误差

 function ZP_Int = Storage_Iteration_P2_220713(InpPar, ZP_Int, xlcs, j1, xcs_t, Con_Int, Q_temp, ZP_IntError_Nor, ZP_IntError_NorTan, drtaT)

% global InpPar.NM_FW InpPar.Nw InpPar.N_ConPatch InpPar.Type_Side InpPar.Exp_WS InpPar.Exp_DummyRail
% global ZP_Int xlcs j1 
       
ZP_Int.Zwy{xcs_t,1} = xlcs;
ZP_Int.Zwy{xcs_t,2} = j1;
ZP_Int.Zwy{xcs_t,3} = drtaT;
ZP_Int.Zwy{xcs_t,4} = Con_Int.Zwy;
ZP_Int.Zwy{xcs_t,5} = Con_Int.Zsd;
ZP_Int.Zwy{xcs_t,6} = Con_Int.Zjsd;
ZP_Int.Zwy{xcs_t,7} = Con_Int.Pxt;
ZP_Int.Zwy{xcs_t,8} = ZP_IntError_Nor(:,end);
ZP_Int.Zwy{xcs_t,9} = ZP_IntError_NorTan(:,end);
ZP_Int.Zwy{xcs_t,10} = Q_temp;

for i1 = 1:1:InpPar.Nw
    T1 = InpPar.Exp_WS{i1};
    for i2 = 1:1:InpPar.N_ConPatch
        T2 = InpPar.Exp_DummyRail{i2};
        ZP_Int.Dis_Rail.(T1).(T2){xcs_t,:} = Con_Int.Dis_Rail.([T1,'_',T2]);
        ZP_Int.Vel_Rail.(T1).(T2){xcs_t,:} = Con_Int.Vel_Rail.([T1,'_',T2]);
        ZP_Int.Acc_Rail.(T1).(T2){xcs_t,:} = Con_Int.Acc_Rail.([T1,'_',T2]);
    end
end

for i1 = 1:1:InpPar.Nw
    T1 = InpPar.Exp_WS{i1};
    ZP_Int.Con_WR_Geometry.(T1){xcs_t,1} = Con_Int.WR_Geometry.(T1);
    for i3 = 1:1:length(InpPar.Type_Side)
        T3 = InpPar.Type_Side{i3};
        len = size( Con_Int.Wheel_2.(T3).(T1), 2 );
        ZP_Int.Con_Wheel_2.(T3).(T1)(xcs_t,1:len) = Con_Int.Wheel_2.(T3).(T1);
        ZP_Int.Con_Wheel_1_Ia.(T3).(T1)(xcs_t,1:len) = Con_Int.Wheel_1_Ia.(T3).(T1);
        ZP_Int.Con_Wheel_1_Ib.(T3).(T1)(xcs_t,1:len) = Con_Int.Wheel_1_Ib.(T3).(T1);
        if isfield(Con_Int.Ver_Pen_postive_start.(T3), T1)
            ZP_Int.Con_Ver_Pen_postive_start.(T3).(T1)(xcs_t,1:len) = Con_Int.Ver_Pen_postive_start.(T3).(T1);
            ZP_Int.Con_Ver_Pen_postive_end.(T3).(T1)(xcs_t,1:len) = Con_Int.Ver_Pen_postive_end.(T3).(T1);
        end
        ZP_Int.Con_Rail_1.(T3).(T1)(xcs_t,1:len) = Con_Int.Rail_1.(T3).(T1);
        ZP_Int.Con_Rail_2.(T3).(T1)(xcs_t,1:len) = Con_Int.Rail_2.(T3).(T1);
        ZP_Int.Con_NF.(T3).(T1)(xcs_t,1:len) = Con_Int.NF.(T3).(T1);
        ZP_Int.Con_Vjd.(T3).(T1)(xcs_t,1:len) = Con_Int.Vjd.(T3).(T1);
        ZP_Int.Con_Vjd_r.(T3).(T1)(xcs_t,1:len) = Con_Int.Vjd_r.(T3).(T1);
        ZP_Int.Con_Vsdc.(T3).(T1)(xcs_t,1:len) = Con_Int.Vsdc.(T3).(T1);
        ZP_Int.Con_RelVel.(T3).(T1)(xcs_t,1:len) = Con_Int.RelVel.(T3).(T1);
        ZP_Int.Con_RHLv.(T3).(T1)(xcs_t,1:len) = Con_Int.RHLv.(T3).(T1);
        ZP_Int.Con_RHXS.(T3).(T1)(xcs_t,1:len) = Con_Int.RHXS.(T3).(T1);
        ZP_Int.Con_Prh.(T3).(T1)(xcs_t,1:len) = Con_Int.Prh.(T3).(T1);
        ZP_Int.Con_Prhx_T.(T3).(T1)(xcs_t,1:len) = Con_Int.Prhx_T.(T3).(T1);
        ZP_Int.Con_Prhxf_T.(T3).(T1)(xcs_t,1:len) = Con_Int.Prhxf_T.(T3).(T1);
       
        if InpPar.NM_FW > 0
            ZP_Int.FW_Con_ShapeFun_FW.(T3).XOY.(T1)(xcs_t,1:len) = Con_Int.ShapeFun_FW.(T3).XOY.(T1);
            ZP_Int.FW_Con_ShapeFun_FW.(T3).Z.(T1)(xcs_t,1:len) = Con_Int.ShapeFun_FW.(T3).Z.(T1);
        end
    end
end
