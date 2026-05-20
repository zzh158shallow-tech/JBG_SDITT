function fun_txt2prr(filename,targ)

    filename_prr = [filename '.prr'];
    
    A{1}='! *************************************************';
    A{2}='! ***        SIMPACK Rail-Wheel Profile         ***';
    A{3}='! *************************************************';
    A{4}=strcat('! File Name      :',num2str(filename));
    A{5}='!';
    A{6}='! This is a SIMPACK example rail profile.';
    A{7}='! This file should not be modified.';
    A{8}='! The settings in this file may slightly change in future releases.';
    A{9}='! It is recommended to use a copy of this file in order to avoid any result changes.';
    A{10}='!';
    A{11}='! Refer to the SIMPACK Documentation for creating your own rail profile files.';
    A{12}='! *************************************************';
    A{13}='! ***               Header Data               ***';
    A{14}='! *************************************************';
    A{15}='  header.begin';
    A{16}='    version       =  1                       ! Version flag';
    A{17}='    type          =  0                       ! 0=rail profile, 1=wheel profile';
    A{18}='  header.end';
    A{19}='! *************************************************';
    A{20}='! ***    Processing Data and Profile Points     ***';
    A{21}='! *************************************************';
    A{22}='  spline.begin';
    A{23}='    approx.smooth  = +0.000000000000000e+00   ! Approximation smoothing value';
    A{24}='    file           = ''-''                      ! Original data file';
    A{25}='    file.mtime     =  0                       ! Time of last modification in seconds since (00:00:00 1970-01-01 UTC)';
    A{26}='    comment        = ''UIC 60 Rail Profile''    ! Comment';
    A{27}='    type           =  0                       ! Original data file type';
    A{28}='    point.dist.min = +0.000000000000000e+00   ! 1: Minimum point distance... (in lengths unit)';
    A{29}='    shift.y        = +0.000000000000000e+00   ! 2: y shift of data ......... (in lengths unit)';
    A{30}='    shift.z        = +0.000000000000000e+00   ! 2: z shift of data ......... (in lengths unit)';
    A{31}='    rotate         = +0.000000000000000e+00   ! 3: Rotation angle about x .. (in angles unit)';
    A{32}='    bound.y.min    = +1.000000000000000e+00   ! 4: y boundary min .......... (in lengths unit)';
    A{33}='    bound.y.max    = +0.000000000000000e+00   ! 4: y boundary max .......... (in lengths unit)';
    A{34}='    bound.z.min    = +1.000000000000000e+00   ! 4: z boundary min .......... (in lengths unit)';
    A{35}='    bound.z.max    = +0.000000000000000e+00   ! 4: z boundary max .......... (in lengths unit)';
    A{36}='    mirror.y       =  0                       ! 5: y mirror mode (0=no mirroring; 1=mirrored)';
    A{37}='    mirror.z       =  0                       ! 5: z mirror mode (0=no mirroring; 1=mirrored)';
    A{38}='    inversion      =  0                       ! 6: Inversion of data order (0=no inversion; 1=inversion)';
    A{39}='    units.len      = ''m''                      ! 7: Unit name for lengths';
    A{40}='    units.ang      = ''rad''                    ! 7: Unit name for angles';
    A{41}='    units.len.f    = +1.000000000000000e+00   ! 7: Unit factor for lengths (l[User] / l[m])';
    A{42}='    units.ang.f    = +1.000000000000000e+00   ! 7: Unit factor for angles (a[User] / a[rad])';
    A{43}='    point.begin';
    A{44}='      ! ';
    A{45}='      ! y value (lengths unit)  z value (lengths unit)    weight (default=1.0)';
    A{46}='      ! ----------------------  ----------------------  ----------------------';
    A{47}='    point.end';
    A{48}='  spline.end';
    A{49}='! *************************************************';
    A{50}='! ***                 E n d                     ***';
    A{51}='! *************************************************';
    
    fid=fopen(filename_prr,'wt');
    for j=1:46
        fprintf(fid,'%s\n',A{j});
    end
    fprintf(fid,'%12.8f %12.8f\n',targ');
    for j=47:51
        fprintf(fid,'%s\n',A{j});
    end
    fclose(fid);