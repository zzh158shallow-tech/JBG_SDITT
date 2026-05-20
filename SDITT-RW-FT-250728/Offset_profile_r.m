function [profile_Radius, profile_r, profile_Front, profile_Rear, profile_Front_d1, profile_Rear_d1, dY, dZ] = ...
    Offset_profile_r(InpPar, Opt_Wheelside, i11, i2, Ori_prr, Mileage, Dis_Rail, profile_ori_inp, Dis_TIrr_temp)
      
% global InpPar.TIrr InpPar.Exp_WS InpPar.N_ConPatch
      
% profile_ori_inp = Profile_ProCS_inp;
% Ori_prr = Par_Track.Ori_prr;

if strcmp(Opt_Wheelside,'L')
    kk = -1;
elseif strcmp(Opt_Wheelside,'R')
    kk = 1;
end

i_wheel = InpPar.N_ConPatch*(i11-1)+i2;
profile_ori = profile_ori_inp.([InpPar.Exp_WS{i11},'_Profile']);
profile_ori_Radius = profile_ori_inp.([InpPar.Exp_WS{i11},'_Radius']);
profile_ori_Front = profile_ori_inp.([InpPar.Exp_WS{i11},'_FrontProfile']);
profile_ori_Rear = profile_ori_inp.([InpPar.Exp_WS{i11},'_RearProfile']);
profile_ori_Front_d1 = profile_ori_inp.([InpPar.Exp_WS{i11},'_FrontProfile_d1']);
profile_ori_Rear_d1 = profile_ori_inp.([InpPar.Exp_WS{i11},'_RearProfile_d1']);

profile_Radius = profile_ori_Radius;
profile_Front_d1 = profile_ori_Front_d1;
profile_Rear_d1 = profile_ori_Rear_d1;

dY = Dis_Rail(i_wheel,2) + Dis_TIrr_temp{1,i2}(2);
dZ = 0.6 + Dis_Rail(i_wheel,3) + Dis_TIrr_temp{1,i2}(3);

% i_wheel
% Dis_Rail(i_wheel,2)
% Dis_TIrr_temp{1,i2}(2)
% dY

if ~isempty(profile_ori)
    profile_Radius(:,1) = kk*(profile_ori_Radius(:,1)+(1.435/2+Ori_prr)) + dY;
    profile_r(:,1) = kk*(profile_ori(:,1)+(1.435/2+Ori_prr)) + dY;
    profile_r(:,2) = profile_ori(:,2) + dZ;
    profile_Front(:,1) = kk*(profile_ori_Front(:,1)+(1.435/2+Ori_prr)) + dY;
    profile_Front(:,2) = profile_ori_Front(:,2) + dZ;
    profile_Rear(:,1) = kk*(profile_ori_Rear(:,1)+(1.435/2+Ori_prr)) + dY;
    profile_Rear(:,2) = profile_ori_Rear(:,2) + dZ;
else
    profile_r = [];
    profile_Radius = [];
    profile_Front = [];
    profile_Rear = [];
    profile_Front_d1 = [];
    profile_Rear_d1 = [];
end
if ~isempty(profile_ori_Front_d1)
    profile_Front_d1(:,2) = kk*(profile_ori_Front_d1(:,2)+(1.435/2+Ori_prr)) + dY;
end
if ~isempty(profile_ori_Rear_d1)
    profile_Rear_d1(:,2)  = kk*(profile_ori_Rear_d1(:,2)+(1.435/2+Ori_prr)) + dY;
end
