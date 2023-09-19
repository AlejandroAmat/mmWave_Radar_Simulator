function  showPCloud(Pcloud, range)

    variable_library_radar;
    pointsTotal = Pcloud;
    
    %a = length(range());
    %for i=1:length(range)
    pointsTotal = [pointsTotal; [0,0,1]];
    pointsTotal = [pointsTotal; [0,range,1]];
    pointsTotal = [pointsTotal; [-range/2,range/2,1]];
    pointsTotal = [pointsTotal; [range/2,range/2,1]];

    %end
 
    for i=1:40
        for j=1:40
            point=[array_x_m(i,j), 0, array_z_m(i,j)];
            pointsTotal = [pointsTotal; point];
        end
    end
    ptCloud = pointCloud(pointsTotal);
    

    


    pcshow(ptCloud);
    title('Original Pointcloud')