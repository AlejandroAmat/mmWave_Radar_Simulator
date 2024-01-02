function [radar_heatmap, visible_cart_v] = main_snr_IRS_random
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
    ptCloudO = pointCloud(pointsTotal);
    
    


     directory_path = './Sim_Walking';
     txt_files = dir(fullfile(directory_path, '*.txt'))
     
     for p = 1:numel(txt_files)
        % Get the filename
        file_name = txt_files(p).name;
        
        % Extract the first num_chars characters
        range = str2double(file_name(1:2));
        users = file_name(12);
        modal = file_name(6:11);
        
    
        
     
     new_folder=['../results/','IMAGERandomIRSRot', '-Pow', num2str(Tx_power),'dB-Range', num2str(range), 'm-Users', num2str(users), '-', modal];
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


    for CAD_idx = 1:600
         close all;
       for k=1:4
            x(k)=1/100* randi([-range*50, range* 50]);
            y(k)=1/100* randi([0, range*100]);
            z(k)=1/10* randi([0, 20]);
        end
     

        radarTX = [x(1), 0, z(1);
           x(2), range, z(2);
           -range/2, y(3), z(3);
          range/2, y(4), z(4)];  
          % translationy = 0.1*randi([1,range*2]);
          % translationx = 0.1*randi([-1*(range/2),range/2]);
    
       for U = 1:users

        translationx =x_data(U,CAD_idx);
        translationy = y_data(U,CAD_idx) + range/2; 
         randomang= rand()*360; 
        transf(U,:)={translationx, translationy,randomang};
            
               
         randomang= rand()*360;      
        rotationAngles2 = [90 0 0];
        rotationAngles = [90 0 randomang];
        tform = rigidtform3d(rotationAngles,[translationx translationy 0]);
        tform2 = rigidtform3d(rotationAngles2,[translationx translationy 0]);
        
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
        end
         
         
       end
         showPCloud(ptCloud.Location, range)
         title('Original Pointcloud')
       
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
        car_v.N_pt = length(ptCloud.Location);
        car_v.cart_v = ptCloud.Location;
        car_v.lim = [min(ptCloud.Location);max(ptCloud.Location)]; % find the limits in all three dimensions 
        [bbox_x, bbox_y, bbox_z] = meshgrid(car_v.lim(:,1),car_v.lim(:,2),car_v.lim(:,3)); % 8 vertices of the bounding box of the point cloud
        car_v.bbox = [bbox_x(:), bbox_y(:), bbox_z(:)]; 
        %clear cart_v bbox N_pt car_idx;
        car1_v_origin = car_v;
        % car_v = car_v_struct;
        % car_v.CAD_idx = CAD_idx;
        % car_v.N_pt = length(cart_v);
        % car_v.cart_v = cart_v;
        % car_v.lim = [min(cart_v);max(cart_v)]; % find the limits in all three dimensions 
        % [bbox_x, bbox_y, bbox_z] = meshgrid(car_v.lim(:,1),car_v.lim(:,2),car_v.lim(:,3)); % 8 vertices of the bounding box of the point cloud
        % car_v.bbox = [bbox_x(:), bbox_y(:), bbox_z(:)]; 
        % clear cart_v bbox N_pt car_idx;
        % car1_v_origin = car_v;
         
       
        

        for ks = 1:1
            
            car_scene_v = car1_v_origin;

            % %% Rotate     
            % car_scene_v.rotate = rotate_ang(randi(length(rotate_ang))); % randomly select a rotation angle and store it in the pc structure
            % car_scene_v.rotate = mod(car_scene_v.rotate*(randi(1)*2-1),180);
            % 
            % 
            % % inline function for 2D rotation
            % rotate2d =  @(x, M) (x(:, 1:2) * M);
            % 
            % rotate_angle_rad = car_scene_v.rotate/180*pi;
            % rotation_matrix = [cos(rotate_angle_rad), -sin(rotate_angle_rad); sin(rotate_angle_rad), cos(rotate_angle_rad)]; % create rotation matrix
            % 
            % car_scene_v.cart_v(:,1:2) = rotate2d(car_scene_v.cart_v, rotation_matrix); % rotate the point cloud 
            % car_scene_v.bbox(:,1:2) = rotate2d(car_scene_v.bbox, rotation_matrix); % rotate the bounding box
            % car_scene_v.lim = [min(car_scene_v.cart_v);max(car_scene_v.cart_v)]; % update the limits in all three dimensions
            % 
            % %% Translation
            % translate_x_rng = (translate_lim(1,1) - car_scene_v.lim(1,1)):translate_x_res:(translate_lim(1,2) - car_scene_v.lim(2,1)); % range of translation along x axis
            % translate_y_rng = (translate_lim(2,1) - car_scene_v.lim(1,2)):translate_y_res:(translate_lim(2,2) - car_scene_v.lim(2,2)); % range of translation along y axis
            % 
            % translate_x = translate_x_rng(randi(length(translate_x_rng))); % randomly select a translation distance along x axis
            % translate_y = translate_y_rng(randi(length(translate_y_rng))); % randomly select a translation distance along y axis
            % translate_z = -1250; % translate the point cloud -1250mm to compensate for the height of our radar 
            % 
            % % translate
            % car_scene_v.translate = [translate_x, translate_y, translate_z]; % store translation information in the pc structure
            % car_scene_v.cart_v = car_scene_v.cart_v + car_scene_v.translate; % translate the point cloud
            % car_scene_v.bbox = car_scene_v.bbox + car_scene_v.translate; % translate the bounding box
            % car_scene_v.lim = [min(car_scene_v.cart_v);max(car_scene_v.cart_v)]; % update the limits in all three dimensions
            % 
            % % convert unit from mm to m
            % car_scene_v.cart_v = car_scene_v.cart_v/1000; 
            % car_scene_v.bbox = car_scene_v.bbox/1000; 
            % 
            % % Visulize the rotated and translated point cloud
            % % figure; 
            % % cart_v_plot = car_scene_v.cart_v; % downsampling when plotting
            % % scatter3(cart_v_plot(:,1),cart_v_plot(:,2),cart_v_plot(:,3),10,'filled','k'); hold on;
            % % scatter3(car_scene_v.bbox(:,1), car_scene_v.bbox(:,2),car_scene_v.bbox(:,3),'r'); hold on;
            % % xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); axis equal;
            % % xlim([-3 3]);
            % % ylim([0 12]);
            % % set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18

            % pcshow(car_scene_v.cart_v);
            % title('Rotated and Translated Pointcloud')
            % 
            %% Modle radar point reflectors in the scene
            %% Modle radar point reflectors in the scene
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
            

            % pcshow(visible_cart_v);
            % title('occlusion')

            % showPCloud(reflector_cart_v, range)
            % title('Reflector Model')
            % 
             reflector_cart_v_d = pcdownsample(pointCloud(reflector_cart_v),'gridAverage',0.015);
            showPCloud(reflector_cart_v_d.Location, range)
             title('Reflector Model')

               
                reflector_cart_v_d= reflector_cart_v_d.Location
             
                
                for Tx=1:nTx

                 signal_array = simulate_radar_signal(reflector_cart_v_d, radarTX(Tx,:));

                %% Radar signal processing, generating 3D radar heatmaps
                 radar_heatmap = radar_dsp(signal_array);

                if(Tx==1)
                    radar_heatmap_top = squeeze(max(radar_heatmap,[],3));
                    save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(Tx), 'top.mat'], 'radar_heatmap_top');
                    figure
                    imagesc(radar_heatmap_top);    
                    set(gca,'XDir','reverse')
                    set(gca,'YDir','normal')
                    colormap jet; caxis([0 1e11]);
                    colorbar;
                    set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
    
                    saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(Tx), 'top.jpg'])


                radar_heatmap_front = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(Tx), 'front.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(Tx), 'front.jpg'])


                 fprintf(fileID, '%s--->[', num2str(CAD_idx)); 
                    
                            for i = 1:3
                                fprintf(fileID, '%.7f ', (transf{U, i})); 
                            end

                            fprintf(fileID, ']\n');

                   
                end

                if (Tx==2)
                    
                radar_heatmap_back = squeeze(max(radar_heatmap,[],1));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(Tx), 'back.mat'], 'radar_heatmap_back');
                figure;
                imagesc(radar_heatmap_back.');    
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30)
                saveas(gcf,['../results/', new_folder, '/', num2str(CAD_idx), '-', num2str(Tx), 'Back.jpg'])
                end

                if (Tx==3)
                    radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(Tx), 'side1.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                set(gca,'XDir','reverse')
                colormap jet; caxis([0 1e11]);
               colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(Tx), 'side1.jpg'])


                end
               
                if (Tx==4)

                     radar_heatmap_front = squeeze(max(radar_heatmap,[],2));
                     
                save(['../results/',new_folder,'/',num2str(CAD_idx),'-',num2str(Tx), 'side2.mat'], 'radar_heatmap_front');
                figure;
                imagesc(radar_heatmap_front.');    
                
                colormap jet; caxis([0 1e11]);
                colorbar;
                set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(Tx), 'side2.jpg'])
                


                end
                    
                    

                    % i=0;
                    % k=0;
                    % while (i<length(SNR))
                    %     SNR_subset = SNR((1+200*k):200*(k+1) );
                    %     SNR_subset_mean = mean(SNR_subset);
                    %     SNR_subset_std = std(SNR_subset);
                    %     SNR_output(CAD_idx,Tx+4*k) = SNR_subset_mean;
                    %     SD_output(CAD_idx,Tx+4*k) = SNR_subset_std;
                    %     i=i+200;
                    %     k=k+1;
                    % end
                    
                %     n=0;   
                %     l=0;
                %     i=0;
                %     K=1;
                %     fprintf(fileID, '%s--->[', num2str(CAD_idx)); 
                %     while (n<length(ptCloud.Location))
                %         SNR_subset = SNR((1+200*l):200*(l+1) );
                %         SNR_subset_mean = mean(SNR_subset);
                %         SNR_subset_std = std(SNR_subset);
                % 
                % 
                %         fprintf(fileID, '%s,%s: [SNR: %d, SD: %d] , ', num2str(K),num2str(mod(i,4)+1),SNR_subset_mean, SNR_subset_std); 
                % 
                % 
                % 
                % 
                %         l=l+1;
                %         n=n+200;
                %         i=i+1;
                %         if(users==1)
                %          if(i==4)
                %          fprintf(fileID, ']\n');
                %          end
                %         elseif(users==2)
                %          if(i==4)
                % 
                %             K=K+1;
                %             fprintf(fileID, ']\n --->[');
                %          elseif(i==8)
                %             fprintf(fileID, ']\n');
                %          end
                %         elseif(users==3)
                %             if(i==4)
                % 
                %             K=K+1;
                %             fprintf(fileID, ']\n --->[');
                %             elseif(i==8)
                % 
                %             K=K+1;
                %             fprintf(fileID, ']\n --->[');
                %             elseif(i==12)
                %                 fprintf(fileID, ']\n');
                %             end
                %         end
                % 
                %     end
                % 
                % %% Simualte received radar signal in the receiver antenna array            
                % % signal_array = simulate_radar_signal(reflector_cart_v_d, TX_pos(Tx,:));
                % % 
                % % %% Radar signal processing, generating 3D radar heatmaps
                % % radar_heatmap = radar_dsp(signal_array);
                % % 
                % % 
                % % % Visulize the radar heatmap top view
                % % radar_heatmap_top = squeeze(max(radar_heatmap,[],3));
                % % figure
                % % imagesc(radar_heatmap_top);    
                % % set(gca,'XDir','reverse')
                % % set(gca,'YDir','normal')
                % % colormap jet; caxis([0 1e11]);
                % % xlabel('Range'); ylabel('Azimuth');
                % % set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                % % 
                % % saveas(gcf,['../results/',new_folder,'/', num2str(CAD_idx),'-',num2str(Tx), 'Top.jpg'])
                % % 
                % % % Visulize the radar heatmap front view
                % % radar_heatmap_front = squeeze(max(radar_heatmap,[],1));
                % % figure;
                % % imagesc(radar_heatmap_front.');    
                % % set(gca,'XDir','reverse')
                % % colormap jet; caxis([0 1e11]);
                % % xlabel('Azimuth'); ylabel('Elevation');
                % % set(gca,'FontSize',30) % Creates an axes and sets its FontSize to 18
                % % saveas(gcf,['../results/',new_folder,'/' num2str(CAD_idx),'-',num2str(Tx), 'Front.jpg'])
                % % 
                % % if (Tx==1)
                % %     fprintf(fileID, '%s--->[', num2str(CAD_idx)); 
                % %     for U = 1:users
                % %         if(U==1) 
                % %             for i = 1:2
                % %                 fprintf(fileID, '%.7f ', (transf{U, i})); 
                % %             end
                % % 
                % %             fprintf(fileID, ']');
                % %             if(users ~= 1)
                % %                 fprintf(fileID, ', [');
                % %             end
                % % 
                % %         elseif(U==2)
                % %             for i = 1:2
                % %                 fprintf(fileID, '%.7f ', (transf{U, i})); 
                % %             end
                % % 
                % %             fprintf(fileID, ']');
                % %             if(users ~= 2)
                % %                 fprintf(fileID, ', [');
                % %             end
                % %         else
                % %             for i = 1:2
                % %                 fprintf(fileID, '%.7f ', (transf{U, i})); 
                % %             end
                % % 
                % %             fprintf(fileID, ']');
                % %         end 
                % %     end
                % %     fprintf(fileID, '\n');
                % % 
                % % end
                % % save(['../results/',new_folder,'/','HeatMap',num2str(CAD_idx), '.mat'], 'radar_heatmap');
            end
            
            %count_num = count_num + 1
        end
    end
    % save(['../results/',new_folder,'/','SNR.mat'], 'SNR_output');
    % save(['../results/',new_folder,'/','SD.mat'], 'SD_output');
    end
end
