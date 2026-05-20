%% 轮轨力对柔性轨道（有限单元法）系统做功
function Pxt = WR_Force_FT_FEM(Pxt, Pjcc, Pjch, Prhxf, Con_WS, ShapeFunction)

global xlcs Nw N_ConPatch Type_Rail Expression_WS Type_Side
                        
%% 计算 Pxt_Track (DOF_Rail*1)
for i1 = 1:1:Nw             %%% 轮对数 1-4
    if xlcs <= 3 && isstruct(Con_WS.FF)
        for i2 = 1:3:4      %%% 接触对数 1, 2
            i_contact = N_ConPatch*(i1-1)+i2;
            F_Z = -(Pjcc(i_contact,1)+Prhxf(i_contact,3));
            F_Y = -(Pjch(i_contact,1)+Prhxf(i_contact,2));
            F_X = - Prhxf(i_contact,1);
            ShapeFunction_Y = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Y']);
            ShapeFunction_Z = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Z']);
            row_F_Lat = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Y']);
            row_F_Ver = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Z']);
            Pxt(row_F_Lat,1) = Pxt(row_F_Lat,1) + F_Y * ShapeFunction_Y';
            Pxt(row_F_Ver,1) = Pxt(row_F_Ver,1) + F_Z * ShapeFunction_Z';
        end
    else
        Con_str = Con_WS.(Expression_WS{i1});
        for wheelside = 1:1:2
            Normal_Force = Con_str.(['Normal_Force_',Type_Side{wheelside}]);
            Prhxf_temp = Con_str.(['Prhxf_',Type_Side{wheelside}]);
            for k = 1:1:size(Normal_Force,1)
                i2 = Normal_Force(k,4);
                F_Z = -(Normal_Force(k,3)+Prhxf_temp(k,3));
                F_Y = -(Normal_Force(k,2)+Prhxf_temp(k,2));
                F_X = - Prhxf_temp(k,1);
                ShapeFunction_Y = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Y']);
                ShapeFunction_Z = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Z']);
                row_F_Lat = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Y']);
                row_F_Ver = ShapeFunction.([Expression_WS{i1},'_',Type_Rail{i2,1},'_Mapping_DynStatus_Z']);
                Pxt(row_F_Lat,1) = Pxt(row_F_Lat,1) + F_Y * ShapeFunction_Y';
                Pxt(row_F_Ver,1) = Pxt(row_F_Ver,1) + F_Z * ShapeFunction_Z';
            end
        end
    end
end


