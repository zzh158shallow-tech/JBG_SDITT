function mod_t = Set_mod_t(InpPar, j1, xlcs)

if strcmp(InpPar.Type_Track,'Co-Running')
    Coff = 5;
else
    Coff = 1;
end

if (j1 >50 && j1<60) || (j1 >102 && j1<108)
    mod_t = 25;
else
    mod_t = 50;
end

mod_t = mod_t*Coff;