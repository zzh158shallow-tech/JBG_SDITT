%% Shape Functions-Beam188

function [ShapeFunction, RailBeam_Motion] = Cal_ShapeFunction_Beam188_FWV(InpPar, Par_Vehicle, j1)

% global InpPar.Nw InpPar.N_ConPatch InpPar.Pos_Node InpPar.N_Node j1 InpPar.Distance_Vehicle InpPar.Type_Rail InpPar.Exp_WS InpPar.RailBeam

RailBeam_Motion = struct;

for i1 = 1:1:InpPar.Nw
    T1 = InpPar.Exp_WS{i1};

    for i2 = 1:1:InpPar.N_ConPatch
        T2 = InpPar.Type_Rail{i2,1};
        T2t = InpPar.Exp_DummyRail{1,i2};
        Mileage  = j1-Par_Vehicle.Distance_Vehicle(i1);

        Pos_Node_temp = InpPar.Pos_Node.(T2);
        Pos_Node_Length_temp = InpPar.Pos_Node.([T2, '_Length']);
        m = find(Mileage-Pos_Node_temp(:,1+1)>=0, 1, 'last');

        if ~isempty(m) && m<InpPar.N_Node.(T2)
            %% A. ShapeFunction
            % Her. + Linear
            x = Mileage - Pos_Node_temp(m,2);
            a = Pos_Node_Length_temp(m,1);
            sigma = (x/a);

%             ShapeFunction.([T1,'_',T2,'_Y']) = [((1-3*sigma^2+2*sigma^3)+(1-sigma))/2,  (sigma-2*sigma^2+sigma^3)*a, (sigma^2*(3-2*sigma)+sigma)/2,   (sigma^3-sigma^2)*a];
%             ShapeFunction.([T1,'_',T2,'_Z']) = [((1-3*sigma^2+2*sigma^3)+(1-sigma))/2, -(sigma-2*sigma^2+sigma^3)*a, (sigma^2*(3-2*sigma)+sigma)/2, -(sigma^3-sigma^2)*a];

            ShapeFunction.([T1,'_',T2,'_Y']) = [(1-3*sigma^2+2*sigma^3),  (sigma-2*sigma^2+sigma^3)*a, sigma^2*(3-2*sigma),   (sigma^3-sigma^2)*a];
            ShapeFunction.([T1,'_',T2,'_Z']) = [(1-3*sigma^2+2*sigma^3), -(sigma-2*sigma^2+sigma^3)*a, sigma^2*(3-2*sigma), -(sigma^3-sigma^2)*a];
            ShapeFunction.([T1,'_',T2,'_ROTY']) = [1-sigma,  sigma];
            ShapeFunction.([T1,'_',T2,'_ROTZ']) = [1-sigma,  sigma];

            len = 0;
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Y']) = [len+(m-1)*4+1; len+(m-1)*4+4; len+m*4+1; len+m*4+4];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Z']) = [len+(m-1)*4+2; len+(m-1)*4+3; len+m*4+2; len+m*4+3];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTY']) = [len+(m-1)*4+3; len+m*4+3];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTZ']) = [len+(m-1)*4+4; len+m*4+4];

            %% B. RailBeam_Motion
            % RailBeam_Motion.Pos_Z /  Vel_Z
            if strcmp(T2t, 'R2') && (Mileage<=InpPar.RailBeam.Vel_Z.R2_zjg(end,1) || Mileage>=InpPar.RailBeam.Vel_Z.R2_zjg(1,1))
                T = 'R2_zjg';
                Tar_Pos = InpPar.RailBeam.Pos_Z.(T);
                Tar_Vel = InpPar.RailBeam.Vel_Z.(T);
                RailBeam_Motion.Pos_Z.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
                RailBeam_Motion.Vel_Z.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
            end

%             if strcmp(T2t, 'R3') && (Mileage<=InpPar.RailBeam.Vel_Z.R3(end,1) || Mileage>=InpPar.RailBeam.Vel_Z.R3(1,1))
%                 T = 'R3';
%                 Tar_Pos = InpPar.RailBeam.Pos_Z.(T);
%                 Tar_Vel = InpPar.RailBeam.Vel_Z.(T);
%                 RailBeam_Motion.Pos_Z.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
%                 RailBeam_Motion.Vel_Z.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
%             end
            
            % RailBeam_Motion.Pos_Y /  Vel_Y
%             if strcmp(T2t, 'R1')
%                 % qjbg
%                 if Mileage<=50+1e-4
%                     RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0;
%                     RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 0;
%                 elseif Mileage>50+1e-4 && Mileage<=50+5.161
%                     RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 26.8e-3/5.161*(Mileage-50);
%                     RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 26.8e-3/5.161*InpPar.Vlc;
%                 else
%                     Phi_0 = 26.8e-3/5.161;
%                     R = 1100;
%                     x_0 = R*sin(Phi_0)-5.161;
%                     RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0.012+R*(1-cos((Mileage-50+x_0)/R));
%                     RailBeam_Motion.Vel_Y.(T2t)(i1,1) = sin((Mileage-50+x_0)/R)*InpPar.Vlc;
%                 end
%                 RailBeam_Motion.Win.(T2t)(i1,1) = 1;
%             elseif strcmp(T2t, 'R2') && (Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1) || Mileage>=InpPar.RailBeam.Pos_Y.R2_zgyg(1,1))
%                 % zjg / zgyg
%                 if Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1)
%                     T = 'R2_zjg';
%                     Tar_Win = InpPar.RailBeam.Win.(T);
%                     RailBeam_Motion.Win.(T2t)(i1,1) = interp1(Tar_Win(:,1), Tar_Win(:,2), Mileage, 'linear');
%                 elseif Mileage>=InpPar.RailBeam.Pos_Y.R2_zgyg(1,1)
%                     T = 'R2_zgyg';
%                     RailBeam_Motion.Win.(T2t)(i1,1) = 1;
%                 end
%                 Tar_Pos = InpPar.RailBeam.Pos_Y.(T);
%                 Tar_Vel = InpPar.RailBeam.Vel_Y.(T);
%                 RailBeam_Motion.Pos_Y.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
%                 RailBeam_Motion.Vel_Y.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
%             elseif strcmp(T2t, 'R3') && Mileage<=50+52.890+2.170
%                 % cxg
%                 RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 35.7e-3/2.170*(Mileage-50-52.890);      % Âú¶¥¿íÖÁcxgÀíÂÛ¼â¶Ë
%                 RailBeam_Motion.Vel_Y.(T2t)(i1,1) = (71.3/2-11.0)*1e-3/(2.170-0.699)*InpPar.Vlc;
%                 RailBeam_Motion.Win.(T2t)(i1,1) = interp1(InpPar.RailBeam.Win.R3(:,1), InpPar.RailBeam.Win.R3(:,2), Mileage, 'linear');
%             else
%                 RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0;
%                 RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 0;
%                 RailBeam_Motion.Win.(T2t)(i1,1) = 1;
%             end

        else
            ShapeFunction.([T1,'_',T2,'_Y']) = [];
            ShapeFunction.([T1,'_',T2,'_Z']) = [];
            ShapeFunction.([T1,'_',T2,'_ROTY']) = [];
            ShapeFunction.([T1,'_',T2,'_ROTZ']) = [];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Y']) = [];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Z']) = [];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTY']) = [];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTZ']) = [];
        end

    end
end

