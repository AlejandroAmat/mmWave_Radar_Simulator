function [radar_heatmap, visible_cart_v] = main_snr_IRS_exhaustive
    % Copyright (c) 2018-2020 Junfeng Guan, Sohrab Madani, Suraj Jog, Saurabh Gupta, 
    % Haitham Hassanieh, University of Illinois at Urbana-Champaign
    % 
    % Permission is hereby granted, free of charge, to any person obtaining a copy
    % of this software and associated documentation files (the "Software"), to deal
    % in the Software without restriction, including without limitation the rights
    % to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    % copies of the Software, and to permit persons to whom the Software is
    % furnished to do so, subject to the following conditions:
    % 
    % The above copyright notice and this permission notice shall be included in
    % all copies or substantial portions of the Software.
    % 
    % THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    % IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    % FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    % AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    % LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    % OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    % THE SOFTWARE.

    close all; clear; clc;
    addpath('functions');

    variable_library;
    variable_library_radar;
    %Import an STL mesh, returning a PATCH-compatible face-vertex structure
    % fv = stlread('../../w.stl');
    nTx = 4;    
    fv = stlread('../../w.stl');
      
    points = fv.Points;
    [points_size, cdd] = size(points);
    
    


    %Linear interpolation of the points
    pointsX = interp1(1:points_size, points(:,1),linspace(1,points_size,points_size*10));
    pointsY = interp1(1:points_size, points(:,2),linspace(1,points_size,points_size*10));
    pointsZ = interp1(1:points_size, points(:,3),linspace(1,points_size,points_size*10));
    pointsTotal = [pointsX' pointsY' pointsZ']/6;
    ptCloudD = pointCloud(pointsTotal);



     pointsTotal = [0,0,0];

    for x=1:10
        for z=1:20
            point=[0.1*x-0.5,-0.50,z*0.1];
            pointsTotal = [pointsTotal; point];
        end
    end

    
    for x=1:10
        for z=1:20
            point=[0.1*x-0.5,0.50,z*0.1];
            pointsTotal = [pointsTotal; point];
        end
    end

    for y=1:10
        for z=1:20
            point=[-0.40,0.1*y-0.6,z*0.1];
            pointsTotal = [pointsTotal; point];
        end
    end
  
    for y=1:10
        for z=1:20
            point=[0.50,0.1*y-0.5,z*0.1];
            pointsTotal = [pointsTotal; point];
        end
    end
    ptCloud = pointCloud(pointsTotal(2:801, :));
    %ptCloud = pointCloud(pointsTotal);
    ptCloudO = ptCloud;
    


     directory_path = './Sim_Walking';
     txt_files = dir(fullfile(directory_path, '*.txt'))
     
     for p = 1:numel(txt_files)
        % Get the filename
        file_name = txt_files(p).name;
        
        % Extract the first num_chars characters
        range = str2double(file_name(1:2));
        users = file_name(12);
        modal = file_name(6:11);
        
    
        
     
     new_folder=['../results/','IMAGINGExhaustiveRot', '-Pow', num2str(Tx_power),'dB-Range', num2str(range), 'm-Users', num2str(users), '-', modal];
     mkdir( new_folder);  
     fileID = fopen([new_folder,'/Transformations.txt'],"w");
     % 
    % Perform Delaunay triangulation
    % load('../../CAD_model_1.mat');
    % 
    % pcshow(cart_v);
    % title('STL Occluded Point Cloud');
    fid = fopen((['./Sim_Walking/', file_name]), 'r');
    users=str2double(users);
    x_coordinates = cell(users,600);
    y_coordinates = cell(users, 600);

    x_data = zeros(users,600);
    y_data = zeros(users, 600);

    transf = cell(users, 3);

 for U = 1:users
    line = fgetl(fid);
    
    % Extract data within curly braces
    data_str = extractBetween(line, '{', '}');
    
    % Split data into individual coordinate sets
    coordinates = extractBetween(data_str{1}, '[', ']');
    
    % Process each coordinate set
    x_coordinates(U,:) =  split(coordinates(1), ',');
    y_coordinates(U,:) =  split(coordinates(2), ',');
    
    x_data(U,:) = cellfun(@str2double, x_coordinates(U,:));
    y_data(U,:) = cellfun(@str2double, y_coordinates(U,:));
    

 end

 SNR_output = ones(600,4*users);  
 SD_output = ones(600,4*users);  
 finalSNR= ones(600,4,users);
 finalSD= ones(600,4,users);
 positions = ones(600,4,users,3)


 X_Coord = [linspace(-range/2, range/2,100); linspace(-range/2, range/2,100); linspace(-range/2, -range/2,100); linspace(range/2, range/2,100)];
 Y_Coord = [linspace(0 ,0,100); linspace(range ,range,100);linspace(0 ,range,100); linspace(0 ,range,100)];
 Z_Coord = linspace(0,2,10); 

    for CAD_idx = 1:600
         close all;
       
    
       for U = 1:users

        translationx =x_data(U,CAD_idx);
        translationy = y_data(U,CAD_idx) + range/2; 
        randomang= rand()*360;  
        transf(U,:)={translationx, translationy,randomang};
            
               
              
        rotationAngles = [90 0 0];
        rotationAngle2 = [90 0 randomang];
        tform = rigidtform3d(rotationAngles,[translationx translationy 0]);
        tform2 = rigidtform3d(rotationAngle2,[translationx translationy 0]);
        
        if(U>1)
             if(U==2)
                ptCloud2 = pctransform(ptCloudO,tform);
                mergedPoints = [ptCloud.Location; ptCloud2.Location];

                % Create a new point cloud from the merged points
                ptCloud = pointCloud(mergedPoints);
             else
                ptCloud3 = pctransform(ptCloudO,tform);
                mergedPoints = [ptCloud.Location; ptCloud3.Location];

                % Create a new point cloud from the merged points
                ptCloud = pointCloud(mergedPoints);
             end
        else
            ptCloud = pctransform(ptCloudO,tform);
            ptCloudDEG = pctransform(ptCloudD, tform2)
        end
         
         
       end
         
       
        % load the surface model
        
        % 
        % CAD models are loaded as point clouds of size N_pt by 3, where N_pt
        % is the number of points and 3 values are the cartesian coordinates
        % unit is mm
        
        % Visulize the original point cloud
%         figure; 
%         cart_v_plot = cart_v;
% %         cart_v_plot = datasample(cart_v, 1000); % downsampling when plotting
%         scatter3(cart_v_plot(:,1),cart_v_plot(:,2),cart_v_plot(:,3),10,'filled','k'); hold on;
%         xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)'); axis equal;
%         set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
        
        %store point cloud in pc (point cloud) structure
        car_v = car_v_struct;
        car_v.CAD_idx = CAD_idx;
        car_v.N_pt = length(ptCloudDEG.Location);
        car_v.cart_v = ptCloudDEG.Location;
        car_v.lim = [min(ptCloudDEG.Location);max(ptCloudDEG.Location)]; % find the limits in all three dimensions 
        [bbox_x, bbox_y, bbox_z] = meshgrid(car_v.lim(:,1),car_v.lim(:,2),car_v.lim(:,3)); % 8 vertices of the bounding box of the point cloud
        car_v.bbox = [bbox_x(:), bbox_y(:), bbox_z(:)]; 
        %clear cart_v bbox N_pt car_idx;
        car1_v_origin = car_v;
        car_scene_v = car1_v_origin;
        % car_v = car_v_struct;
        % car_v.CAD_idx = CAD_idx;
        % car_v.N_pt = length(cart_v);
        % car_v.cart_v = cart_v;
        % car_v.lim = [min(cart_v);max(cart_v)]; % find the limits in all three dimensions 
        % [bbox_x, bbox_y, bbox_z] = meshgrid(car_v.lim(:,1),car_v.lim(:,2),car_v.lim(:,3)); % 8 vertices of the bounding box of the point cloud
        % car_v.bbox = [bbox_x(:), bbox_y(:), bbox_z(:)]; 
        % clear cart_v bbox N_pt car_idx;
        % car1_v_origin = car_v;
         
       SD_stored = ones(users,4,100,10); 
       SNR_stored = ones(users,4,100,10);
      
       maximSNR = ones(4,users);
       min_index = ones(4,users,2);
       
       [visible_cart_v ] = remove_occlusion(car_scene_v); % remove occluded body of the car
            try
                reflector_cart_v = model_point_reflector(visible_cart_v,car_scene_v.bbox); % model point reflectors that reflect back to the radar receiver
            catch
                continue;
            end

            if isempty(reflector_cart_v)
                continue;
            end
            
            % Visulize the radar point reflectors
            % figure; 
            % cart_v_plot = reflector_cart_v; % downsampling when plotting
            % scatter3(cart_v_plot(:,1),cart_v_plot(:,2),cart_v_plot(:,3),10,'filled','k'); hold on;
            % xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); axis equal;
            % xlim([-3 3]);
            % ylim([0 12]);
            % set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
            

            pcshow(visible_cart_v);
            title('occlusion')

            % showPCloud(reflector_cart_v, range)
            % title('Reflector Model')
            % 
             reflector_cart_v_d = pcdownsample(pointCloud(reflector_cart_v),'gridAverage',0.015);
            showPCloud(reflector_cart_v_d.Location, range)
             title('Reflector Model')

               
                reflector_cart_v_d= reflector_cart_v_d.Location






 
                for Tx=1:nTx
                    
                    for xy=1:100
                        for z=1:10

                            location = [X_Coord(Tx,xy),Y_Coord(Tx,xy),Z_Coord(z)];
                            SNR = SNRRand(users, Tx, ptCloud.Location, location);
                            
        
                            i=0;
                            k=0;
                            while (i<length(SNR))
                                SNR_subset = SNR((1+200*k):200*(k+1) );
                                SNR_subset_mean = mean(SNR_subset);
                                SNR_subset_std = std(SNR_subset);
                                
                                if(SNR_subset_mean > maximSNR(Tx, k+1))
                                    maximSNR(Tx,k+1) = SNR_subset_mean;
                                    min_index(Tx, k+1,1) = xy;
                                    min_index(Tx, k+1,2) = z;

                                end
                                SNR_stored(k+1,Tx, xy, z) = SNR_subset_mean;
                                SD_stored(k+1,Tx, xy, z) = SNR_subset_std;
                                % SNR_output(CAD_idx,Tx+4*k) = SNR_subset_mean;
                                % SD_output(CAD_idx,Tx+4*k) = SNR_subset_std;
                                i=i+200;
                                k=k+1;
                            end
                        end
                    end

                    
              
                end

                

                for us=1:users
                    a=maximSNR(us,:);
                    [minSNR, minIndex]= min(maximSNR(:,us));
                    
                    signal_array = simulate_radar_signal(reflector_cart_v_d, [X_Coord(minIndex,min_index(minIndex,us,1)),Y_Coord(minIndex,min_index(minIndex,us,1)),Z_Coord(min_index(minIndex,us,2))]);

                     %% Radar signal processing, generating 3D radar heatmaps
                    radar_heatmap = radar_dsp(signal_array);

                   radar_heatmap_top = squeeze(max(radar_heatmap,[],3));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(minIndex), 'top.mat'], 'radar_heatmap_top');
                figure
                imagesc(radar_heatmap_top);    
                set(gca,'XDir','reverse')
                set(gca,'YDir','normal')
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18

                saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(minIndex), 'top.jpg'])

                    
                 fprintf(fileID, '%s--->[', num2str(CAD_idx)); 
                    
                            for i = 1:3
                                fprintf(fileID, '%.7f ', (transf{U, i})); 
                            end

                            fprintf(fileID, ']\n');

                   
                

                 if (minIndex==2)
                    
                radar_heatmap_back = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(minIndex), 'back.mat'], 'radar_heatmap_back');
                figure;
                imagesc(radar_heatmap_back.');    
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30)
                saveas(gcf,['../results/', new_folder, '/', num2str(CAD_idx), '-', num2str(minIndex), 'Back.jpg'])
                

                 elseif (minIndex==3)
                    radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(minIndex), 'side1.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
               colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(minIndex), 'side1.jpg'])


                
               
                elseif (minIndex==4)

                     radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                     
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(minIndex), 'side2.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(minIndex), 'side2.jpg'])
                


                 else
                
                     radar_heatmap_front = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(minIndex), 'front.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(minIndex), 'front.jpg'])
                end



                    for i=1:nTx
                        if(i~=minIndex)
                        
                            
                           absoluteDifferences = abs(SNR_stored(us,i,:,:) - minSNR);
                           [minDifference, minIndexTemp] = min(absoluteDifferences(:)); 
                
                           caux= int8(minIndexTemp/100);
                           if(caux==0)
                               c=1;
                           else

                           c= caux;
    
                           end

                           raux=mod(minIndexTemp,100);
                           if(raux==0)
                               r=100;
                           else

                           r= raux;
    
                           end
    
                           % The element in the matrix that is most similar to 'value' is matrix(row, col)

                            signal_array = simulate_radar_signal(reflector_cart_v_d, [ X_Coord(i,r), Y_Coord(i,r),Z_Coord(c)]);

                              %% Radar signal processing, generating 3D radar heatmaps
                             radar_heatmap = radar_dsp(signal_array);


                                if (i==2)
                    
                radar_heatmap_back = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(i), 'back.mat'], 'radar_heatmap_back');
                figure;
                imagesc(radar_heatmap_back.');    
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30)
                saveas(gcf,['../results/', new_folder, '/', num2str(CAD_idx), '-', num2str(i), 'Back.jpg'])
                

                 elseif (i==3)
                    radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(i), 'side1.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
               colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(i), 'side1.jpg'])


                
               
                elseif (i==4)

                     radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                     
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(i), 'side2.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(i), 'side2.jpg'])
                


                 else
                
                     radar_heatmap_front = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(i), 'front.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(i), 'front.jpg'])
                end






                           closestValue = SNR_stored(us,i,r,c)

                           finalSNR(CAD_idx,i,us)=closestValue;
                           finalSD(CAD_idx,i,us)=SD_stored(us,i,r,c);
                           positions(CAD_idx,i,us,1) = X_Coord(i,r);
                           positions(CAD_idx,i,us,2) = Y_Coord(i,r);
                           positions(CAD_idx,i,us,2) = Y_Coord(i,r);
                           positions(CAD_idx,i,us,3) = Z_Coord(c);


                           
                              
    
    
            
                        else
                            % finalSNR(CAD_idx,i,us)=minSNR;
                            % finalSD(CAD_idx,i,us)=SD_stored(us,i,min_index(minIndex,us,1),min_index(minIndex,us,2));
                            % 
                            % positions(CAD_idx,i,us,1) = X_Coord(i,min_index(minIndex,us,1));
                            % positions(CAD_idx,i,us,2) = Y_Coord(i,min_index(minIndex,us,1));
                            % positions(CAD_idx,i,us,3) = Z_Coord(min_index(minIndex,us,2));

                        end
    
    
    
                    end
                end

                % a=positions(CAD_idx,:,:,:);
                % 
                % showPCloudEx(ptCloud.Location, users, CAD_idx,positions)
                % title('Original Pointcloud')
                % 
                % 
            
                
        
    end

    end
end
