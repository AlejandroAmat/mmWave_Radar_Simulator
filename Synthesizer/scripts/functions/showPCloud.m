function  showPCloud(Pcloud)

    variable_library_radar;
    pointsTotal = Pcloud;
    newPoint = [2, 2, 4];
    newPoint2 = [-2,-2,0]

    pointsTotal = [pointsTotal; newPoint];
    pointsTotal = [pointsTotal; newPoint2];
    pointsTotal = [pointsTotal; TX_pos];
    ptCloud = pointCloud(pointsTotal);
    

    


    pcshow(ptCloud);
    title('Original Pointcloud')