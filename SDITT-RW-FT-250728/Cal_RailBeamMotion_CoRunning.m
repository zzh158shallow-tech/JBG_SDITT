%% Shape Functions-Beam188

function RailBeam_Motion = Cal_RailBeamMotion_CoRunning(InpPar, Par_Vehicle, j1)

for i1 = 1:1:InpPar.Nw
%     T1 = InpPar.Exp_WS{i1};    
    for i2 = 1:1:InpPar.N_ConPatch
%         T2 = InpPar.Type_Rail{i2,1};
        T2t = InpPar.Exp_DummyRail{1,i2};
        Mileage  = j1-Par_Vehicle.Distance_Vehicle(i1);        

        % C. InpPar.RailBeam
        if strcmp(T2t, 'R2') && (Mileage<=InpPar.RailBeam.Vel_Z.R2_zjg(end,1) || Mileage>=InpPar.RailBeam.Vel_Z.R2_zjg(1,1))
            T = 'R2_zjg';
            Tar_Pos = InpPar.RailBeam.Pos_Z.(T);
            Tar_Vel = InpPar.RailBeam.Vel_Z.(T);
            RailBeam_Motion.Pos_Z.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
            RailBeam_Motion.Vel_Z.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
        end

        if strcmp(T2t, 'R1')
            % qjbg
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
            RailBeam_Motion.Win.(T2t)(i1,1) = 1;
%         elseif strcmp(T2t, 'R2') && Mileage>=InpPar.RailBeam.Vel_Y.R2_zgyg(1,1)
        elseif strcmp(T2t, 'R2') && (Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1) || Mileage>=InpPar.RailBeam.Pos_Y.R2_zgyg(1,1))
            % zjg / zgyg
%             if Mileage<=50+10.981
%                 RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 43.7e-3/10.981*(Mileage-50);
%                 RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 43.7e-3/10.981*InpPar.Vlc;
            if Mileage<=InpPar.RailBeam.Vel_Y.R2_zjg(end,1)
                T = 'R2_zjg';
                Tar_Win = InpPar.RailBeam.Win.(T);
                RailBeam_Motion.Win.(T2t)(i1,1) = interp1(Tar_Win(:,1), Tar_Win(:,2), Mileage, 'linear');
            elseif Mileage>=InpPar.RailBeam.Pos_Y.R2_zgyg(1,1)
                T = 'R2_zgyg';
                RailBeam_Motion.Win.(T2t)(i1,1) = 1;
            end
            Tar_Pos = InpPar.RailBeam.Pos_Y.(T);
            Tar_Vel = InpPar.RailBeam.Vel_Y.(T);
            RailBeam_Motion.Pos_Y.(T2t)(i1,1) = interp1(Tar_Pos(:,1), Tar_Pos(:,2), Mileage, 'linear');
            RailBeam_Motion.Vel_Y.(T2t)(i1,1) = interp1(Tar_Vel(:,1), Tar_Vel(:,2), Mileage, 'linear');
        elseif strcmp(T2t, 'R3') && Mileage<=50+52.890+2.170
            % cxg
            RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 35.7e-3/2.170*(Mileage-50-52.890);      % Âú¶¥¿íÖÁcxgÀíÂÛ¼â¶Ë
            RailBeam_Motion.Vel_Y.(T2t)(i1,1) = (71.3/2-11.0)*1e-3/(2.170-0.699)*InpPar.Vlc;
%             RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 35.7e-3/2.170*InpPar.Vlc;
            RailBeam_Motion.Win.(T2t)(i1,1) = interp1(InpPar.RailBeam.Win.R3(:,1), InpPar.RailBeam.Win.R3(:,2), Mileage, 'linear');
        else
            RailBeam_Motion.Pos_Y.(T2t)(i1,1) = 0;
            RailBeam_Motion.Vel_Y.(T2t)(i1,1) = 0;
            RailBeam_Motion.Win.(T2t)(i1,1) = 1;
        end

    end
end