function  showPCloudEx(Pcloud,users, CAD_idx,range)

    variable_library_radar;
    pointsTotal = Pcloud;
    
    %a = length(range());
    for i=1:users
        pointsTotal = [pointsTotal; [range(CAD_idx,1,i,1),range(CAD_idx,1,i,2),range(CAD_idx,1,i,3)]];
        pointsTotal = [pointsTotal; [range(CAD_idx,2,i,1),range(CAD_idx,2,i,2),range(CAD_idx,2,i,3)]];
        pointsTotal = [pointsTotal; [range(CAD_idx,3,i,1),range(CAD_idx,3,i,2),range(CAD_idx,3,i,3)]];
        pointsTotal = [pointsTotal; [range(CAD_idx,4,i,1),range(CAD_idx,4,i,2),range(CAD_idx,4,i,3)]];

    end
 
    for i=1:40
        for j=1:40
            point=[array_x_m(i,j), 0, array_z_m(i,j)];
            pointsTotal = [pointsTotal; point];
        end
    end
    ptCloud = pointCloud(pointsTotal);
    

    


    pcshow(ptCloud);
    title('Original Pointcloud')