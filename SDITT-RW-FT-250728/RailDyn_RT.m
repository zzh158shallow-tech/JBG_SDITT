%% 计算车轮下钢轨的位移、速度、加速度
function [Dis_Rail, Vel_Rail, Acc_Rail] = RailDyn_RT(InpPar, Zwy, Zsd, Zjsd, Tar_SIP)

Nw = InpPar.Nw;
N_ConPatch = InpPar.N_ConPatch;
DOF_OneWS = InpPar.N_track/InpPar.Nw;

Dis_Rail = zeros(Nw*N_ConPatch,6);      % 钢轨接触点处 X、Y、Z 方向位移（列） m
Vel_Rail = zeros(Nw*N_ConPatch,6);      % 钢轨接触点处 X、Y、Z 方向（列）速度 m/s
Acc_Rail = zeros(Nw*N_ConPatch,6);      % 钢轨接触点处 X、Y、Z 方向（列）加速度 m/s^2
      
if Tar_SIP == 1
    Range_ConPatch = [1,2,3];       % L1, R1, R2
    Range_DOF = [1,4,3];
elseif Tar_SIP == 2
    Range_ConPatch = [1,3,4];       % L1, R2, R3
    Range_DOF = [1,4,3];
end

for i1 = 1:1:Nw         %%% 轮对数
    for i2 = 1:1:length(Range_DOF)
        i_wheel = N_ConPatch*(i1-1)+Range_ConPatch(i2);
        i_num = DOF_OneWS*(i1-1)+2*(Range_DOF(i2)-1);
        Dis_Rail(i_wheel,2) = Zwy(i_num+2,4);     % 钢轨横向位移
        Dis_Rail(i_wheel,3) = Zwy(i_num+1,4);     % 钢轨垂向位移
        Vel_Rail(i_wheel,2) = Zsd(i_num+2,4);     % 钢轨横向速度
        Vel_Rail(i_wheel,3) = Zsd(i_num+1,4);     % 钢轨垂向速度
        Acc_Rail(i_wheel,2) = Zjsd(i_num+2,4);    % 钢轨横向加速度
        Acc_Rail(i_wheel,3) = Zjsd(i_num+1,4);    % 钢轨垂向加速度
    end
end