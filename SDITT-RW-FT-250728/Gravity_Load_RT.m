function Pxt_Gravity = Gravity_Load_RT(InpPar, Par_Vehicle, Rail, Baseplate, Tar_SIP)
                       
Pxt_Gravity = zeros(InpPar.N_track+InpPar.NM_FW*4+InpPar.N_RV,1);
% 车辆系统重力做功
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+1,1)  = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+6,1)  = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+11,1) = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+16,1) = 9.8*Par_Vehicle.Mw;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+21,1) = 9.8*Par_Vehicle.Mb;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+26,1) = 9.8*Par_Vehicle.Mb;
Pxt_Gravity(InpPar.N_track+InpPar.NM_FW*4+31,1) = 9.8*Par_Vehicle.Mc;

% 刚性道岔重力做功
if strcmp(InpPar.Type_Track,'Co-Running')
    if Tar_SIP==1
    Mass_DOF = [repmat(2.5,1,4), ...
                          repmat(Rail.Mass.zjbg,1,1), repmat(Rail.Mass.zjg_zgyg,1,1), repmat(Rail.Mass.zjg_zgyg,1,1), repmat(Rail.Mass.qjbg,1,1), ...
                          Baseplate.Mass.Switch_L, Baseplate.Mass.Switch_R];
    elseif Tar_SIP==2
    Mass_DOF = [repmat(2.5,1,4), ...
                           repmat(Rail.Mass.zjbg(2),1,1), repmat(Rail.Mass.zjg_zgyg(2),1,1), repmat(Rail.Mass.cxg(2),1,1), repmat(Rail.Mass.zjg_zgyg(2),1,1), ...
                           Baseplate.Mass.Crossing_L(2), Baseplate.Mass.Crossing_Center(2)];

    end
    for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
        Pxt_Gravity(18*(i1-1)+1,1) = 9.8*Mass_DOF(1);
        Pxt_Gravity(18*(i1-1)+3,1) = 9.8*Mass_DOF(2);
        Pxt_Gravity(18*(i1-1)+5,1) = 9.8*Mass_DOF(3);
        Pxt_Gravity(18*(i1-1)+7,1) = 9.8*Mass_DOF(4);
        Pxt_Gravity(18*(i1-1)+9,1) = 9.8*Mass_DOF(5);
        Pxt_Gravity(18*(i1-1)+11,1) = 9.8*Mass_DOF(6);
        Pxt_Gravity(18*(i1-1)+13,1) = 9.8*Mass_DOF(7);
        Pxt_Gravity(18*(i1-1)+15,1) = 9.8*Mass_DOF(8);
        Pxt_Gravity(18*(i1-1)+17,1) = 9.8*Mass_DOF(9);
        Pxt_Gravity(18*(i1-1)+18,1) = 9.8*Mass_DOF(10);
    end
end


% if strcmp(InpPar.Type_Track,'Co-Running')
%     for i1 = 1:1:InpPar.Nw           %%% 轮对数 1-4
%         M_Rail_sum = [Par_Track.Mr1, Par_Track.Mr2, Par_Track.Mr3, Par_Track.Mr4];
%         Pxt_Gravity(11*(i1-1)+1,1) = 9.8*M_Rail_sum(1);
%         Pxt_Gravity(11*(i1-1)+3,1) = 9.8*M_Rail_sum(2);
%         Pxt_Gravity(11*(i1-1)+5,1) = 9.8*M_Rail_sum(3);
%         Pxt_Gravity(11*(i1-1)+7,1) = 9.8*M_Rail_sum(4);
%         Pxt_Gravity(11*(i1-1)+9,1) = 9.8*Ms;
%     end
% end
