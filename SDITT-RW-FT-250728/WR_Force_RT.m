%% 轮轨力对刚性轨道系统做功
function Pxt = WR_Force_RT(InpPar, Pxt, Pjcc, Pjch, Prhxf, Con_WS, xlcs, Tar_SIP)

DOF_OneWS = InpPar.N_track/InpPar.Nw;
if Tar_SIP == 1
    Range_ConPatch = [1,2,3];       % L1, R1, R2
    Range_DOF = [1,4,3];
elseif Tar_SIP == 2
    Range_ConPatch = [1,3,4];       % L1, R2, R3
    Range_DOF = [1,4,3];
end

for i1 = 1:1:InpPar.Nw           %%% 轮对数
    for i2 = 1:1:length(Range_DOF)
        i_contact = InpPar.N_ConPatch*(i1-1)+Range_ConPatch(i2);
        pos_DOF = DOF_OneWS*(i1-1)+2*(Range_DOF(i2)-1);
        Pxt(pos_DOF+1,1) = Pxt(pos_DOF+1,1) - Pjcc(i_contact,1);     %%% Z方向法向力
        Pxt(pos_DOF+2,1) = Pxt(pos_DOF+2,1) - Pjch(i_contact,1);     %%% Y方向法向力
        Pxt(pos_DOF+1,1) = Pxt(pos_DOF+1,1) - Prhxf(i_contact,3);   %%% Z方向蠕滑力
        Pxt(pos_DOF+2,1) = Pxt(pos_DOF+2,1) - Prhxf(i_contact,2);   %%% Y方向蠕滑力        
    end
end


% for i1 = 1:1:InpPar.Nw             %%% 轮对数 1-4
%     if xlcs <= 3 && ~isfield(Con_WS.FF, 'Normal_Force')
%         for i2 = 1:1:2      %%% 接触对数 1, 2
%             i_contact = InpPar.N_ConPatch*(i1-1)+i2;
%             F_Z = -(Pjcc(i_contact,1)+Prhxf(i_contact,3));
%             F_Y = -(Pjch(i_contact,1)+Prhxf(i_contact,2));
%             F_X = - Prhxf(i_contact,1);
%             Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+2,1) = Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+2,1) + F_Y;
%             Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+1,1) = Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+1,1) + F_Z;
%         end
%     else
%         Con_str = Con_WS.(InpPar.Exp_WS{i1});
%         for wheelside = 1:1:2
%             Normal_Force = Con_str.Normal_Force.(InpPar.Type_Side{wheelside});
%             Prhxf_temp = Con_str.Prhxf_T.(InpPar.Type_Side{wheelside});
%             for k = 1:1:size(Normal_Force,1)
%                 i2 = Normal_Force(k,4);
%                 F_Z = -(Normal_Force(k,3)+Prhxf_temp(k,3));
%                 F_Y = -(Normal_Force(k,2)+Prhxf_temp(k,2));
%                 F_X = - Prhxf_temp(k,1);
%                 Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+2,1) = Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+2,1) + F_Y;
%                 Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+1,1) = Pxt(DOF_OneWS*(i1-1)+2*(i2-1)+1,1) + F_Z;
%             end
%         end
%     end
% end