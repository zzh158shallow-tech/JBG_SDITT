%% STRIPES计算轮轨法向力
% 230623: 划分每个接触点所考虑的渗透量范围，避免重复计算轮轨渗透区

function [NorGap_ConCS, NF_Sum, Area_STRIPES_Sum, Con_STRIPES_oup, N_Stripes_Tar, epsilon] = NF_STRIPES_230623...
                (inp_profile_w_Radius, inp_profile_r_Radius, inp_Con_wheel_1_II, inp_Con_wheel_2_II, inp_Con_rail_1_II, inp_wheel_interp, inp_rail_interp, ...
                inp_Yw_DW, inp_A_DW, inp_B_track, inp_Ver_Pen_a, inp_m, inp_n, inp_Con_A, inp_Con_B, inp_Con_r, BGmn, Er, Vr, N_Stripes_Tar, Med)

% profile_w_Radius.(T), Profile_TrackCS.profile_r_Radius.(T), ...
% Con_wheel_1_II.a.(T), Con_wheel_2_II.a.(T), Con_rail_1_II.a.(T), wheel_interp.(T), rail_interp.(T), ...
% Yw_DW.(T), Yaw_DW.(T), Roll_DW.(T), A_DW.(T), B_track.(T), Ver_Pen_a.(T), m.a.(T), n.a.(T), Con_A.a.(T), Con_B.a.(T), Con_r.a.(T), BGmn, Er, Vr, N_Stripes.(T)

% % Con_wheel_1_IIb, Con_wheel_2_IIb, Con_wheel_1_IIa, Con_wheel_2_II
% inp_profile_w_Radius = profile_w_Radius.R;
% inp_profile_r_Radius = Profile_TrackCS.profile_r_Radius.R;
% inp_Con_wheel_1_II = Con_wheel_1_II.a.R;
% inp_Con_wheel_2_II = Con_wheel_2_II.a.R;
% inp_Con_rail_1_II = Con_rail_1_II.a.R;
% inp_wheel_interp = wheel_interp.R;
% inp_rail_interp = rail_interp.R;
% inp_Yw_DW = Yw_DW.R;
% inp_A_DW = A_DW.R;
% inp_B_track = B_track_a.R;
% inp_Ver_Pen_a = Ver_Pen_a.R;
% inp_m = m.a.R;
% inp_n = n.a.R;
% inp_Con_A = Con_A.a.R;
% inp_Con_B = Con_B.a.R;
% inp_Con_r = Con_r.a.R;
% N_Stripes_Tar = N_Stripes.R;
% Med = 'AB';

Choose_Plot = 0;

%% 修正后法向刚性穿透量
if strcmp(Med, 'A')    
    epsilon = (inp_n.^2) ./ inp_Con_r ./ (1+inp_Con_A./inp_Con_B);          % 只修正 A
elseif strcmp(Med, 'AB')    
    epsilon = (inp_n.^2) ./ inp_Con_r ./ (1+(inp_n./inp_m).^2);                   % 同时修正 A 和 B
end

h_0 = epsilon .* inp_Con_wheel_2_II(:,5);

%% 计算法向间隙和法向渗透量
% clear bools TransMat_track               % Deleted by WX

N_Patch = size(inp_Ver_Pen_a,1);
NorGap_ConCS = cell(N_Patch,6);

bools = cell(N_Patch, 1);                       % Added by WX
TransMat_track = cell(N_Patch, 1);       % Added by WX

for i = 1:1:N_Patch
    
    limit_1 = 1e-3;
    if N_Patch > 1
        if i == 1
            bools_WR = inp_wheel_interp(:,2)<=inp_Ver_Pen_a(i+1,2)-limit_1;
        elseif i == N_Patch
            bools_WR = inp_wheel_interp(:,2)>=inp_Ver_Pen_a(i-1,2)+limit_1;
        else
            bools_WR = inp_wheel_interp(:,2)>=inp_Ver_Pen_a(i-1,2)+limit_1 & inp_wheel_interp(:,2)<=inp_Ver_Pen_a(i+1,2)-limit_1;
        end
    else
        bools_WR = true(size(inp_rail_interp,1),1);
    end
    
    % Con C.S.->Track C.S. Mat.
%     TransMat_track{i,1} = double( subs(T_Con_Track, [Inp_Pos_Yaw, Inp_Pos_Roll, Inp_Ang], [inp_Yaw_DW, inp_Roll_DW, inp_Con_wheel_2_II(i,6)]) );
    TransMat_track{i,1} = inp_B_track{i,1};
    
    % Col 1-2: Track C.S.->Con C.S.: Wheel and Rail interp
    N_NorGap = length(find(bools_WR));
    % 不考虑X坐标
%     NorGap_ConCS{i,1} = ([zeros(N_NorGap,1) inp_wheel_interp(bools_WR,2:3)]-repmat([0 inp_Con_wheel_1_II(i,2:3)],N_NorGap,1)) * (TransMat_track{i,1}\eye(3));
    % 考虑X坐标
%     NorGap_ConCS{i,1} = (inp_wheel_interp(bools_WR,:)-repmat(inp_Con_wheel_1_II(i,1:3),N_NorGap,1)) * (TransMat_track{i,1}\eye(3));
%     NorGap_ConCS{i,2} = ([zeros(N_NorGap,1) inp_rail_interp(bools_WR,:)]-repmat([0 inp_Con_rail_1_II(i,1:2)],N_NorGap,1)) * (TransMat_track{i,1}\eye(3));
    NorGap_ConCS{i,1} = (inp_wheel_interp(bools_WR,:)-repmat(inp_Con_wheel_1_II(i,1:3),N_NorGap,1)) / TransMat_track{i,1};
    NorGap_ConCS{i,2} = ([zeros(N_NorGap,1) inp_rail_interp(bools_WR,:)]-repmat([0 inp_Con_rail_1_II(i,1:2)],N_NorGap,1)) / TransMat_track{i,1};
    
    % NorGap_ConCS, cell(i,6): Con.C.S.下车轮廓形, Con.C.S.下钢轨廓形, 接触点两侧15mm范围内车轮廓形、钢轨廓形, 法向间隙, 法向渗透量
    if abs(inp_Ver_Pen_a(i,2))<0.72
        limit_2 = 0.005;
    else
        limit_2 = 0.015;
    end
    bools{i,1} = abs(NorGap_ConCS{i,1}(:,2))<=limit_2;
%     bools{i,1} = true(N_NorGap,1);                                                    % TEST  !!!!!!
    % Col 3: 接触坐标系中，接触点两侧 y_limit 范围内车轮迹线坐标
    NorGap_ConCS{i,3} = NorGap_ConCS{i,1}(bools{i,1},:);
    % Col 4: 接触坐标系中，钢轨迹线坐标 (插值后)
    NorGap_ConCS{i,4} = [zeros(length(find(bools{i,1})),1), NorGap_ConCS{i,3}(:,2),...
                                         interp1(NorGap_ConCS{i,2}(:,2), NorGap_ConCS{i,2}(:,3), NorGap_ConCS{i,3}(:,2), 'linear')];
    % Col 5: 接触坐标系中，轮轨法向间隙
    NorGap_ConCS{i,5} = [NorGap_ConCS{i,3}(:,2), NorGap_ConCS{i,4}(:,3)-NorGap_ConCS{i,3}(:,3)];
    % Col 6: 虚拟渗透量-删减前
    NorGap_ConCS{i,6} = [NorGap_ConCS{i,3}(:,2), h_0(i,1)-NorGap_ConCS{i,5}(:,2)];

    % Col 7: 虚拟渗透覆盖范围 - Track C.S.
    % Col 8: 避免重复渗透筛选后的虚拟渗透量 - Con C.S.
    bools_pen = NorGap_ConCS{i,6}(:,2)>0;
    p1 = find(bools_pen, 1 );
    p2 = find(bools_pen, 1, 'last' );
    NorGap_ConCS{i,7} = NorGap_ConCS{i,3}([p1, p2], :);
    NorGap_ConCS{i,7} = NorGap_ConCS{i,7}*TransMat_track{i,1} + repmat(inp_Con_wheel_1_II(i,1:3),size(NorGap_ConCS{i,7},1),1);
    if i==1
        NorGap_ConCS{i,8} = NorGap_ConCS{i,6};
    else
        if NorGap_ConCS{i,7}(1,2)>NorGap_ConCS{i-1,7}(2,2)
            NorGap_ConCS{i,8} = NorGap_ConCS{i,6};
        else
            ConPos_lim_Left_TrackCS = NorGap_ConCS{i-1,7}(2,:);
            ConPos_lim_Left_ConCS = (ConPos_lim_Left_TrackCS-inp_Con_wheel_1_II(i,1:3)) / TransMat_track{i,1};
            bools_pen = NorGap_ConCS{i,6}(:,1)>ConPos_lim_Left_ConCS(2);
            NorGap_ConCS{i,8} = NorGap_ConCS{i,6}(bools_pen,:);
        end
    end

    % Col 9-11: 通过单调性来二次筛选
    bools_start = NorGap_ConCS{i,8}(1:end-1,2)<=0 & NorGap_ConCS{i,8}(2:end,2)>=0;  % Start point
    bools_end  = NorGap_ConCS{i,8}(1:end-1,2)>=0 & NorGap_ConCS{i,8}(2:end,2)<=0;  % End point
    % Start points
    NorGap_ConCS{i,9} = NorGap_ConCS{i,8}(find(bools_start),:);
    % Ending points
    NorGap_ConCS{i,10} = NorGap_ConCS{i,8}(find(bools_end),:);
    if N_Patch==1
        bools_pen = true(size(NorGap_ConCS{i,8},1),1);
    else
        if size(NorGap_ConCS{i,9},1)==size(NorGap_ConCS{i,10},1)
            if isempty(find(NorGap_ConCS{i,9}(:,1)>NorGap_ConCS{i,10}(:,1), 1))
                bools_pen = true(size(NorGap_ConCS{i,8},1),1);
            else
                bools_pen = NorGap_ConCS{i,8}(:,1)<NorGap_ConCS{i,9}(end,1) & NorGap_ConCS{i,8}(:,1)>NorGap_ConCS{i,10}(1,1);
            end
        elseif size(NorGap_ConCS{i,9},1)>size(NorGap_ConCS{i,10},1)
            % 右侧无终点的部分都不要
            bools_pen = NorGap_ConCS{i,8}(:,1)<NorGap_ConCS{i,9}(end,1);
        elseif size(NorGap_ConCS{i,9},1)<size(NorGap_ConCS{i,10},1)
            % 左侧无起点的部分都不要
            bools_pen = NorGap_ConCS{i,8}(:,1)>NorGap_ConCS{i,10}(1,1);
        end
    end
    NorGap_ConCS{i,11} = NorGap_ConCS{i,8}(bools_pen,:);

    % 重新计算 Col 7
    bools_pen = NorGap_ConCS{i,11}(:,2)>0;
    p1 = find(bools_pen, 1 );
    p2 = find(bools_pen, 1, 'last' );
    NorGap_ConCS{i,7}(:,2) = NorGap_ConCS{i,11}([p1, p2], 1);
    NorGap_ConCS{i,7}(:,1) = interp1(NorGap_ConCS{i,3}(:,2), NorGap_ConCS{i,3}(:,1), NorGap_ConCS{i,7}(:,2), 'linear');
    NorGap_ConCS{i,7}(:,3) = interp1(NorGap_ConCS{i,3}(:,2), NorGap_ConCS{i,3}(:,3), NorGap_ConCS{i,7}(:,2), 'linear');
    NorGap_ConCS{i,7} = NorGap_ConCS{i,7}*TransMat_track{i,1} + repmat(inp_Con_wheel_1_II(i,1:3),size(NorGap_ConCS{i,7},1),1);
    
end

%% Plot
if Choose_Plot == 1
    i = 1;
    figure(24); clf
    subplot(3,1,1)
    plot(NorGap_ConCS{i,1}(:,2), NorGap_ConCS{i,1}(:,3), NorGap_ConCS{i,2}(:,2), NorGap_ConCS{i,2}(:,3)); hold on
%     plot(NorGap_ConCS{i,3}(:,2), NorGap_ConCS{i,3}(:,3), '--', NorGap_ConCS{i,4}(:,2), NorGap_ConCS{i,4}(:,3), '--'); hold on
    set(gca,'ydir','reverse'); grid on; title('Contact in the Con. C.S.')
    subplot(3,1,2)
    plot(NorGap_ConCS{i,5}(:,1), NorGap_ConCS{i,5}(:,2));
    grid on; title('Normal Gap')
    subplot(3,1,3)
    plot(NorGap_ConCS{i,6}(:,1), NorGap_ConCS{i,6}(:,2), '-'); hold on; 
    plot(NorGap_ConCS{i,8}(:,1), NorGap_ConCS{i,8}(:,2), '--'); hold on; 
    plot(NorGap_ConCS{i,11}(:,1), NorGap_ConCS{i,11}(:,2), '--'); hold on; 
%     plot(NorGap_Con_j(:,1), NorGap_Con_j(:,2), '*'); hold on; 
    set(gca,'ydir','reverse'); grid on; title('Normal Pen');

    i = 2;
    figure(25); clf
    subplot(3,1,1)
    plot(NorGap_ConCS{i,1}(:,2), NorGap_ConCS{i,1}(:,3), NorGap_ConCS{i,2}(:,2), NorGap_ConCS{i,2}(:,3)); hold on
    set(gca,'ydir','reverse'); grid on; title('Contact in the Con. C.S.')
    subplot(3,1,2)
    plot(NorGap_ConCS{i,5}(:,1), NorGap_ConCS{i,5}(:,2));
    grid on; title('Normal Gap')
    subplot(3,1,3)
    plot(NorGap_ConCS{i,6}(:,1), NorGap_ConCS{i,6}(:,2)); hold on; 
    plot(NorGap_ConCS{i,8}(:,1), NorGap_ConCS{i,8}(:,2), '--'); hold on; 
    plot(NorGap_ConCS{i,11}(:,1), NorGap_ConCS{i,11}(:,2), '-'); hold on; 
    set(gca,'ydir','reverse'); grid on; title('Normal Pen');
    
    figure(26); clf
    plot(inp_wheel_interp(:,2), inp_wheel_interp(:,3)); hold on
    plot(Con_wheel_1_Patch(:,2), Con_wheel_1_Patch(:,3), '*'); hold on; 
    set(gca,'ydir','reverse'); grid on; 
    
    figure(26); clf
    plot(profile_w.R(:,1), profile_w.R(:,2)); hold on
    plot(Con_wheel_2_Patch(:,2), Con_wheel_2_Patch(:,3), '*'); hold on; 
    set(gca,'ydir','reverse'); grid on; 
    
    figure(26); clf
    plot(inp_rail_interp(:,1), inp_rail_interp(:,2)); hold on
    plot(Con_rail_1_Patch(:,1), Con_rail_1_Patch(:,2), '*'); hold on; 
    set(gca,'ydir','reverse'); grid on; 
    
    figure(27); clf
    plot(pos_lat, B_j_ori); hold on
    plot(pos_lat, B_j, '--'); hold on; 
    grid on;
    
    figure(28); clf
    plot(pos_lat, A_j); hold on
    
    figure(28); clf
    plot(pos_lat, m_j); hold on
    plot(pos_lat, n_j, '--'); hold on; 
    grid on;
end

%% 计算法向力和接触面积
NF_j = cell(N_Patch,1);
NF_Sum = zeros(N_Patch,6);
Area_STRIPES_j = cell(N_Patch,1);
Area_STRIPES_Sum = zeros(N_Patch,1);
Con_STRIPES_oup = cell(N_Patch,4);

for i = 1:1:N_Patch
%     clear NorGap_Con_j Con_wheel_ConCS Con_rail_ConCS               % Deleted by WX
    NorGap_Con_j=[]; Con_wheel_ConCS=[]; Con_rail_ConCS=[];                                % Added by WX

    bools_Con = NorGap_ConCS{i,11}(:,2)>0;
    if N_Stripes_Tar==0
        N_Stripes_Tar = length(find(bools_Con));
        % Con.C.S.中，每个条带的法向渗透量
        NorGap_Con_j = NorGap_ConCS{i,11}(bools_Con,:);
        % WS.C.S.中，车轮上条带横向坐标
        pos_lat = (NorGap_ConCS{i,11}(bools_Con,1))';
        % dy
        pos = find(bools_Con);
        bools_Con_dy = bools_Con;
        bools_Con_dy([pos(1)-1, pos(end)+1]) = true;
        temp_y = NorGap_ConCS{i,11}(bools_Con_dy,1);        
        dy = diff(temp_y(1:end-1,1))/2 + diff(temp_y(2:end,1))/2;
                
%         % WS.C.S.中，车轮上条带坐标 (不能直接以 bools{i,1} 计算)
%         temp = inp_wheel_interp(bools{i,1},:);
%         Con_wheel_2_PatchTemp = [(temp(p:q,:) - repmat([0,inp_Yw_DW,0],N_Stripes,1))*inv(inp_A_DW)];
%         % Track C.S.中，钢轨上条带坐标
%         temp = inp_rail_interp(bools{i,1},:);
%         Con_rail_1_PatchTemp = temp(p:q,:);
    else
        % WS.C.S.中，车轮上条带横向坐标
        pos_lat = NorGap_ConCS{i,11}(bools_Con,1);
        pos_lat = linspace(pos_lat(1), pos_lat(end), N_Stripes_Tar);
        % Con.C.S.中，每个条带的法向渗透量
        NorGap_Con_j(:,1) = pos_lat;
        NorGap_Con_j(:,2) = interp1(NorGap_ConCS{i,11}(:,1), NorGap_ConCS{i,11}(:,2), pos_lat, 'linear');
        % dy
        dy = repmat( (pos_lat(end)-pos_lat(1))/(N_Stripes_Tar-1), N_Stripes_Tar, 1);
    end
    
    % WS.C.S.中，车轮上条带坐标
    Con_wheel_ConCS(:,2) = pos_lat;
    Con_wheel_ConCS(:,[1,3]) = interp1(NorGap_ConCS{i,1}(:,2), NorGap_ConCS{i,1}(:,[1,3]), pos_lat, 'spline');    
    Con_wheel_1_Patch = Con_wheel_ConCS*TransMat_track{i,1} + repmat(inp_Con_wheel_1_II(i,1:3),N_Stripes_Tar,1);
    Con_wheel_2_Patch = (Con_wheel_1_Patch-repmat([0,inp_Yw_DW,0],N_Stripes_Tar,1)) / inp_A_DW;
    
    % Track C.S.中，钢轨上条带坐标
    Con_rail_ConCS(:,2) = pos_lat;
    Con_rail_ConCS(:,[1,3]) = interp1(NorGap_ConCS{i,2}(:,2), NorGap_ConCS{i,2}(:,[1,3]), pos_lat, 'linear');
    temp = Con_rail_ConCS*TransMat_track{i,1};
    Con_rail_1_Patch = temp(:,2:3) + repmat(inp_Con_rail_1_II(i,1:2),N_Stripes_Tar,1);

    % 计算每个条带的曲率半径等接触参数
    [R_yy_w_j, R_xx_w_j, R_xx_r_j, rou_j] = Re_Radius(inp_profile_w_Radius, inp_profile_r_Radius, Con_wheel_2_Patch, Con_rail_1_Patch);    
    beta_j = acos(rou_j./4.*abs(1./R_yy_w_j-1./R_xx_w_j-1./R_xx_r_j));
    m_j = interp1(BGmn(:,1),BGmn(:,2),beta_j,'linear');
    n_j = interp1(BGmn(:,1),BGmn(:,3),beta_j,'linear');

    % Hertz接触参数 A_j, B_j, Lambda_j
    A_j = 1/2 * 1./R_yy_w_j;
    B_j_ori = 1/2 * (1./R_xx_w_j+1./R_xx_r_j);
    
    % 拟合 B_j
    if length(find(bools_Con))>1
%         [B_j, ~] = csaps(pos_lat, B_j_ori, 1-5e-12, pos_lat);
%         [B_j, ~] = csaps(pos_lat, B_j_ori, 1-5e-11, pos_lat);
%         B_j = B_j';
        B_j = smooth(B_j_ori, 5, 'lowess');
%         B_j = smooth(B_j_ori, 5);
    else
        B_j = B_j_ori;
    end
    
    bools_zero = NorGap_Con_j(:,2)<0;
    
    % 接触椭圆长半轴 a_j
    if strcmp(Med, 'A')        
        A_cj = B_j .* (n_j./m_j).^2;                        % 只修正A
    elseif strcmp(Med, 'AB')        
        A_cj = (A_j+B_j) ./ (1+(m_j./n_j).^2);      % 修正A和B
    end
    
    a_j = zeros(N_Stripes_Tar,1);
    a_j(~bools_zero,1) = (NorGap_Con_j(~bools_zero,2)./A_cj(~bools_zero,1)).^0.5;
    
    % 接触刚度 K_Stiff_j
    if strcmp(Med, 'A')
        K_Stiff_j_ori = (Er*(1+A_j./B_j).*dy) ./ (2*(1-Vr^2)*n_j.^3);                % 只修正A
    elseif strcmp(Med, 'AB')
        K_Stiff_j_ori = (Er*(1+(n_j./m_j).^2).*dy) ./ (2*(1-Vr^2)*n_j.^3);       % 修正A和B
    end
    K_Stiff_j = K_Stiff_j_ori;
    K_Stiff_j(bools_zero) = zeros(length(find(bools_zero)),1);
    
    % 法向力
    NF_j{i,1} = K_Stiff_j .* NorGap_Con_j(:,2);
    NF_Sum(i,5) = sum(NF_j{i,1});
    
    % 接触面积
    Area_STRIPES_j{i,1} = a_j.* dy * 2;
    Area_STRIPES_Sum(i,1) = sum(Area_STRIPES_j{i,1});
    
    % Check Data
    Con_STRIPES_oup{i,1} = Con_wheel_2_Patch;
    Con_STRIPES_oup{i,2} = Con_rail_1_Patch;
    Con_STRIPES_oup{i,3} = [R_yy_w_j, R_xx_w_j, R_xx_r_j, rou_j, beta_j, m_j, n_j, A_j, B_j, K_Stiff_j];
    Con_STRIPES_oup{i,4} = [K_Stiff_j, NorGap_Con_j(:,2), NF_j{i,1}];
    
end


%% Draft
    % 保证单段法向渗透量
%     NorPen_ConCS_d1 = [NorGap_ConCS{i,6}(:,1) gradient(NorGap_ConCS{i,6}(:,2))./gradient(NorGap_ConCS{i,6}(:,1))];
%     bools_check = (NorGap_ConCS{i,6}(1:end-1,2) < 0) & (NorPen_ConCS_d1(1:end-1,2)<0 & NorPen_ConCS_d1(2:end,2)>=0);
%     if ~isempty(find(bools_check))
%         temp = find(bools_check);
%         [~,pos] = min(abs(NorGap_ConCS{i,5}(:,1)));
%         bools_1 = temp<pos;
%         bools_2 = temp>pos;
%         p = 1;
%         q = size(NorGap_ConCS{i,5},1);
%         if ~isempty(find(bools_1))
%             p = max(temp(bools_1));
%         end
%         if ~isempty(find(bools_2))
%             q = min(temp(bools_2));
%         end
%         NorGap_ConCS{i,5} = NorGap_ConCS{i,5}(p:q,:);
%         NorGap_ConCS{i,6} = NorGap_ConCS{i,6}(p:q,:);
%     end
%     figure(31); clf
%     plot(NorPen_ConCS_d1(:,1),NorPen_ConCS_d1(:,2)); grid on

%     bools_check = NorGap_ConCS{i,5}(:,2)<0 & abs(NorGap_ConCS{i,5}(:,1))>0.002;
%     if ~isempty(find(bools_check))
%         temp = find(bools_check);
%         [~,pos] = min(abs(NorGap_ConCS{i,5}(:,1)));
%         bools_1 = temp<pos;
%         bools_2 = temp>pos;
%         p = 1;
%         q = size(NorGap_ConCS{i,5},1);
%         if ~isempty(find(bools_1))
%             p = max(temp(bools_1))+50;
%         elseif ~isempty(find(bools_2))
%             q = min(temp(bools_2))-50;
%         end
%         NorGap_ConCS{i,5} = NorGap_ConCS{i,5}(p:q,:);
%         NorGap_ConCS{i,6} = NorGap_ConCS{i,6}(p:q,:);
%     end
