function  showPCloud(Pcloud, range)

    variable_library_radar;
    pointsTotal = Pcloud;
    
    %a = length(range());
    %for i=1:length(range)
    pointsTotal = [pointsTotal; [range(1,1),range(1,2),range(1,3)]];
    pointsTotal = [pointsTotal; [range(2,1),range(2,2),range(2,3)]];
    pointsTotal = [pointsTotal; [range(3,1),range(3,2),range(3,3)]];
    pointsTotal = [pointsTotal; [range(4,1),range(4,2),range(4,3)]];

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