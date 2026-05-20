%% Shape Functions-Beam188

function [ShapeFunction, RailBeam_Motion] = Cal_ShapeFunction_Beam188(InpPar, Par_Vehicle, j1)

% global InpPar.Nw InpPar.N_ConPatch InpPar.Pos_Node InpPar.N_Node j1 InpPar.Distance_Vehicle InpPar.Type_Rail InpPar.Exp_WS InpPar.RailBeam

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
            % A. ShapeFunction
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
         
%             % Her.
%             x = Mileage - InpPar.Pos_Node_temp(m,2);
%             a = InpPar.Pos_Node_Length_temp(m,1);
%             eval(['ShapeFunction.',T1,'_',T2,'_Y = [1-3*(x/a)^2+2*(x/a)^3, x*(1-2*x/a+(x/a)^2), (x/a)^2*(3-2*x/a), x*((x/a)^2-x/a)];']);
%             eval(['ShapeFunction.',T1,'_',T2,'_Z = [1-3*(x/a)^2+2*(x/a)^3,-x*(1-2*x/a+(x/a)^2), (x/a)^2*(3-2*x/a),-x*((x/a)^2-x/a)];']);
            
%             % 3-D 2-Node Lines (Not Combining Translations and Rotations)
%             a = InpPar.Pos_Node_Length_temp(m,1);
%             x = Mileage - InpPar.Pos_Node_temp(m,2) - a/2;
%             s = x/(a/2);
%             eval(['ShapeFunction.',T1,'_',T2,'_Y = [1/2*(1-s), 0, 1/2*(1+s), 0];']);
%             eval(['ShapeFunction.',T1,'_',T2,'_Z = [1/2*(1-s), 0, 1/2*(1+s), 0];']);

%             % 3-D 2-Node Lines (Combining Translations and Rotations)
%             a = InpPar.Pos_Node_Length_temp(m,1);
%             x = Mileage - InpPar.Pos_Node_temp(m,2) - a/2;
%             s = x/(a/2);
%             eval(['ShapeFunction.',T1,'_',T2,'_Y = [1/2-s/4*(3-s^2),  a/8*(1-s^2)*(1-s), 1/2+s/4*(3-s^2), -a/8*(1-s^2)*(1+s)];']);
%             eval(['ShapeFunction.',T1,'_',T2,'_Z = [1/2-s/4*(3-s^2), -a/8*(1-s^2)*(1-s), 1/2+s/4*(3-s^2),  a/8*(1-s^2)*(1+s)];']);

            % B. DOF Pos
            % 6DOF
%             len = sum(DOF_Rail.Sort(1:i2-1));
%             ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Y']) = [len+(m-1)*6+2; len+(m-1)*6+6; len+m*6+2; len+m*6+6];
%             ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Z']) = [len+(m-1)*6+3; len+(m-1)*6+5; len+m*6+3; len+m*6+5];
            % 4DOF
%             len = sum(DOF_Rail.Sort(1:i2-1));
            % T33
            len = 0;
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Y']) = [len+(m-1)*4+1; len+(m-1)*4+4; len+m*4+1; len+m*4+4];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_Z']) = [len+(m-1)*4+2; len+(m-1)*4+3; len+m*4+2; len+m*4+3];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTY']) = [len+(m-1)*4+3; len+m*4+3];
            ShapeFunction.([T1,'_',T2,'_Mapping_DynStatus_ROTZ']) = [len+(m-1)*4+4; len+m*4+4];
            
            % C. InpPar.RailBeam
%             Range_i3 = {'Pos_Y', 'Vel_Y', 'Pos_Yaw', 'Vel_Yaw', 'Pos_Z', 'Vel_Z', 'Pos_Pitch', 'Vel_Pitch'};
%             for i3 = 1:1:length(Range_i3)
%                 T3 = Range_i3{i3};
%                 Target = InpPar.RailBeam.(T3).(T2);
%                 InpPar.RailBeam_Motion.(T3).(T2)(i1,1:2) = [Mileage, interp1(Target(:,1), Target(:,2), Mileage, 'linear')];
%             end
            % M6
%             InpPar.RailBeam_Motion = [];

            % M7b
%             y1 = [];    y2 = [];
%             T2 = 'R1';
%             Range_Mileage = [49:0.05:58];
%             T2 = 'R2';
%             Range_Mileage = [100:0.05:105];
%             for i0 = 1:1:length(Range_Mileage)
%                 Mileage = Range_Mileage(i0);
                if strcmp(T2t, 'R1')
                    if Mileage<=50+1e-4
                        RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0;
                        RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 0;
                    elseif Mileage>50+1e-4 && Mileage<=50+5.161
                        RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 26.8e-3/5.161*(Mileage-50);
                        RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 26.8e-3/5.161*InpPar.Vlc;
                    else
                        Phi_0 = 26.8e-3/5.161;
                        R = 1100;
                        x_0 = R*sin(Phi_0)-5.161;
                        RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0.012+R*(1-cos((Mileage-50+x_0)/R));
                        RailBeam_Motion.Vel_Y.(T2t)(i1,1) = sin((Mileage-50+x_0)/R)*InpPar.Vlc;
                    end
                elseif strcmp(T2t, 'R2') && Mileage>=InpPar.RailBeam.Vel_Y.R2_zgyg(1,1)
%                 elseif strcmp(T2t, 'R2') && (Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1) || Mileage>102)
%                     if Mileage<=50+10.981
%                         RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 43.7e-3/10.981*(Mileage-50);
%                         RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 43.7e-3/10.981*InpPar.Vlc;
%                     if Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1)
%                         Tar_Pos = InpPar.RailBeam.Pos_Y.R2_zjg;
%                         Tar_Vel = InpPar.RailBeam.Vel_Y.R2_zjg;
%                     elseif Mileage>102
%                         Tar_Pos = InpPar.RailBeam.Pos_Y.R2;
%                         Tar_Vel = InpPar.RailBeam.Vel_Y.R2;
%                     end
                    Tar_Pos = InpPar.RailBeam.Pos_Y.R2_zgyg;
                    Tar_Vel = InpPar.RailBeam.Vel_Y.R2_zgyg;
                    RailBeam_Motion.Pos_Y.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
                    RailBeam_Motion.Vel_Y.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
%                 elseif strcmp(T2t, 'R3') && Mileage<=50+52.890+2.170
%                     RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 35.7e-3/2.170*(Mileage-50-52.890);      % Âú¶¥¿íÖÁcxgÀíÂÛ¼â¶Ë
%                     RailBeam_Motion.Vel_Y.(T2t)(i1,1) = (71.3/2-11.0)*1e-3/(2.170-0.699)*InpPar.Vlc;
% %                     RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 35.7e-3/2.170*InpPar.Vlc;
                else
                    RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0;
                    RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 0;
                end
%                 y1(i0,1) = RailBeam_Motion.Pos_Y.(T2t)(i1,1);
%                 y2(i0,1) = RailBeam_Motion.Vel_Y.(T2t)(i1,1);
%             end
%             figure(99); clf
%             subplot(2,1,1)
%             plot(Range_Mileage, y1); grid on
%             set(gca, 'ydir', 'reverse');
%             subplot(2,1,2)
%             plot(Range_Mileage, y2); grid on

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