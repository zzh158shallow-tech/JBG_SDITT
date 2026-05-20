%% 判断接触位置属于哪个接触�?
function i2 = Judge_i2_ConRail(wheelside, Pos_ConRail_Y, Profile_TrackCS)

% Profile_TrackCS

%% PlainTrack
% if wheelside == 1
%     i2 = 1;
% elseif wheelside == 2
%     i2 = 2;
% end

%% L1, R1, R2, R3
if wheelside == 1
    i2 = 1;
elseif wheelside == 2
    if isempty(Profile_TrackCS.profile_r.R2) && isempty(Profile_TrackCS.profile_r.R3)
        i2 = 2;
    elseif isempty(Profile_TrackCS.profile_r.R1) && isempty(Profile_TrackCS.profile_r.R3)
        i2 = 3;
    elseif isempty(Profile_TrackCS.profile_r.R1) && isempty(Profile_TrackCS.profile_r.R2)
        i2 = 4;
    else
        if ~isempty(Profile_TrackCS.profile_r.R1) && ~isempty(Profile_TrackCS.profile_r.R2)
            if Pos_ConRail_Y > min(Profile_TrackCS.profile_r.R1(:,1))
                i2 = 2;
            else
                i2 = 3;
            end
        end
        if ~isempty(Profile_TrackCS.profile_r.R2) && ~isempty(Profile_TrackCS.profile_r.R3)
            if Pos_ConRail_Y > min(Profile_TrackCS.profile_r.R2(:,1))
                i2 = 3;
            else
                i2 = 4;
            end
        end
    end
end

%% L1, L2, R1, R2
% if wheelside == 1
%     if isempty(Profile_TrackCS.profile_r.L1)        % L2: qjg
%         i2 = 2;
%     elseif isempty(Profile_TrackCS.profile_r.L2)	% L1: zjbg
%         i2 = 1;
%     else
%         if Pos_ConRail_Y < max(Profile_TrackCS.profile_r.L1(:,1))   % L1: zjbg
%             i2 = 1;
%         else                                        % L2: qjg
%             i2 = 2;
%         end
%     end
% elseif wheelside == 2
%     if isempty(Profile_TrackCS.profile_r.R1)        % R2: zjg
%         i2 = 3;
%     elseif isempty(Profile_TrackCS.profile_r.R2)	% R1: qjbg
%         i2 = 4;
%     else
%         if Pos_ConRail_Y > min(Profile_TrackCS.profile_r.R1(:,1))   % R1: qjbg
%             i2 = 4;
%         else                                        % R2: zjg
%             i2 = 3;
%         end
%     end
% end

