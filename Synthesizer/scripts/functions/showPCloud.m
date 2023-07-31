function  showPCloud(Pcloud)

    variable_library_radar;
    pointsTotal = Pcloud;
    newPoint = [8,8, 4];
    newPoint2 = [-8,-8,0]

    pointsTotal = [pointsTotal; newPoint];
    pointsTotal = [pointsTotal; newPoint2];
    pointsTotal = [pointsTotal; TX_pos(1,:)];
    pointsTotal = [pointsTotal; TX_pos(2,:)];
    pointsTotal = [pointsTotal; TX_pos(3,:)];
    pointsTotal = [pointsTotal; TX_pos(4,:)];
    for i=1:40
        for j=1:40
            point=[array_x_m(i,j), 0, array_z_m(i,j)];
            pointsTotal = [pointsTotal; point];
        end
    end
    ptCloud = pointCloud(pointsTotal);
    

    


    pcshow(ptCloud);
    title('Original Pointcloud')