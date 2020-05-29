%% 3DMerge
% Authors: Marcelo Jacinto
%          David Souto
% This project merges 3D point clouds and applies a very simple algorithm
% to detect moving objects.
% Makes use of the vlfeat library
% Also requires the camera calibration files
%% 
clear all;
close all;
clc;

%Enable VLFEAT
run('./../vlfeat-0.9.21/toolbox/vl_setup')

nome_ficheiro = 'newpiv2';

% Get the rgb image list
directory_images=dir(strcat('./',nome_ficheiro,'/*.png'));
if isempty(directory_images)
    directory_images=dir(strcat('./',nome_ficheiro,'/*.jpg'));
end

directory_depth=dir(strcat('./',nome_ficheiro,'/*.mat'));
image_list=cell(length(directory_images),1);
depth_list=cell(length(directory_depth),1);

for i=1:length(directory_images),
    image_list{i}=directory_images(i).name;
    depth_list{i}=directory_depth(i).name;
end

%load the asus camera calibration parameters
%load('cam_params.mat');
load('calib_asus.mat');

numImages = length(directory_images);

%[transforms, objects] = merge3d(depth_list, image_list, cam_params);

%% Getting XYZ of each image and corresponding RGB points

xyz = cell(1, numImages);
rgbd = cell(1, numImages);

for n=1:numImages
  
    %read the image
    img = imread(char(image_list(n)));
    
    %load the correct depth information
    load(char(depth_list(n)))
    
    %convert the scale of the depth
    dep = double(depth_array);%* 1000;
    dep(isnan(dep(:)))=0;
    
    image_size = size(depth_array);
    
    %Get XYZ coordinates for that image and rgbd
    xyz{n} = get_xyzasus(dep(:), [image_size(1,1), image_size(1,2)], 1:640*480, Depth_cam.K,1,0);
    rgbd{n} = get_rgbd(xyz{n}, img, R_d_to_rgb, T_d_to_rgb, RGB_cam.K);
    
    % Visualization
    figure();
    cl=reshape(rgbd{n},480*640,3);
    p=pointCloud(xyz{n},'Color',cl);
    showPointCloud(p)
end

%% Getting the features of each image

% Inicializing 
features_vector = cell(1, numImages);
descriptors_vector = cell(1, numImages);


% Extracting the features and descriptors of every image and storing them 
% in a cell array (using vl_sift)
for n=1:numImages
    
    % Read I(n).
    I = rgbd{1, n};
    
    % Convert image to single to be able to use it with the sift
    % algorithm implemented in VLfeat
    I = im2single(I) ;
    
    % make grayscale if it isnt already
    if size(I,3) > 1
        grayImage = rgb2gray(I) ;
    else
        grayImage = I;
    end
    
    % Detect and extract sift featu res for I(n).
    [features_aux, descriptors_aux] = vl_sift(grayImage, 'Levels', 50) ;
    
    % Save the descriptors and features in a vector of cells to use later
    descriptors_vector{n} = descriptors_aux;
    features_vector{n} = features_aux;
    
    % ----TO BE REMOVED LATER - VISUALIZE THE FEATURES IN THE IMAGE----
     %figure();
     %imshow(I); hold on;
     %perm = randperm(size(features_aux,2)) ;
     % Select the all the features that we want to index and present in the
     % image
     %sel = linspace(1, length(features_aux), length(features_aux));
     % Put the features in the image
     %h1 = vl_plotframe(features_aux(:,sel)) ;
     %h2 = vl_plotframe(features_aux(:,sel)) ;
     %set(h1,'color','k','linewidth',3) ;
     %set(h2,'color','y','linewidth',2) ;
     % -----------------------------------------------------------------
end
%% Calculate the Rotation and Translation between 3D images
minNumPoints = 3;
direct_connections = zeros(numImages, numImages);
vec_ligado_ao_um = zeros(numImages);

% Create the structure to save the rotation and translation between the
% data
transforms = struct('R', 'T');
transforms.R = cell(numImages, numImages);
transforms.T = cell(numImages, numImages);

for j=1:numImages
    for n=j:numImages
        
        if n == j
            transforms.R{n,j} = eye(3,3);
            transforms.T{n,j} = [0; 0; 0];
        end
        
        if n ~= j && abs(j-n) == 1
            [matches, scores] = vl_ubcmatch(descriptors_vector{n},descriptors_vector{j});
            
            % Points in image "n"
           
            X1 = round(features_vector{n}(1:2,matches(1,:)));
            % That match to points in image "j"
            X2 = round(features_vector{j}(1:2,matches(2,:)));
            
            % For each match in 2D, find the corresponding pair of matches in 3D
            X1_3D = zeros(3, length(X1));
            X2_3D = zeros(3, length(X2));
            
            for i=1:length(X1)
                
                %Get the X1_3D
                ind = sub2ind([480 640], X1(2,i), X1(1,i));
                X1_3D(:, i) =  xyz{1,n}(ind,:)';
                
                %Get the X2_3D
                ind = sub2ind([480 640], X2(2,i), X2(1,i));
                X2_3D(:, i) =  xyz{1,j}(ind,:)';
            end
            
            numMatches = size(matches,2);
            
            %RANSAC to filter the outliers in the matches
            score = zeros(1,500);
            ok = cell(1, 500);
            for k=1:500
                % Select 3 random numbers to index the arrays X1 and X2 
                % and choose 3 points in order to calculate the
                % transformation
                subset = vl_colsubset(1:numMatches, minNumPoints);
                
                A = X1_3D(:, subset);
                B = X2_3D(:, subset);
                
                %NOTE - any doesn't support NaN
                %check if there is a point that is all (0,0,0) - if yes,
                %discard this points and keep on going
                if (all(any(A, 2),1) && all(any(B, 2),1))
                    % Calculate the translation and rotation matrix
                    [R_test, T_test] = calculate_tranform(A,B);
                    
                    %Calculate the predicted points
                    X2_3D_ = R_test * X1_3D + repmat(T_test,1, numMatches);
                    
                    %Score the transformation
                    dx = X2_3D_(1,:) - X2_3D(1,:);
                    dy = X2_3D_(2,:) - X2_3D(2,:);
                    dz = X2_3D_(3,:) - X2_3D(3,:);
                 
                    % Get the points in matches for which the geometric
                    % error is inferior to a specified threshold
                    %Basically, mark matches as outliers or inliers
                    ok{1, k} = sqrt((dx.*dx + dy.*dy + dz.*dz)) < 0.05;
                    
                    %Sum the errors in order for the inliers
                    score(1,k) = sum(ok{1,k});
                else
                    score(1,k) = NaN;
                end
            end   
            %Get the one with the max score
            [~,best] = max(score);
            %Get only the inliers for the best transformation found
            ok = ok{best};
            
            inliers = find(ok);
            % Calculate the translation and rotation matrix using all the
            % inliers
            % We only accept transformations that have more than 10 inliers
            % or between adjacent images
            %
            if ( length(inliers) > 10 && abs(j-n) < 3) || (j == n-1 && length(inliers) > 10) || (j == n+1 && length(inliers) > 10)
                [transforms.R{n, j}, transforms.T{n,j}] = calculate_tranform(X1_3D(:,inliers),X2_3D(:,inliers));
            
                if (j == 1)
                    vec_ligado_ao_um(n) = 1;
                end
                % Put the information in connection matrix 
                if (j == n-1 || j == n+1)
                    direct_connections(n,j) = 1/sum(ok);
                %direct_connections(j,n) = 1/sum(ok);
                end
                show_images(image_list, n, j, features_vector, matches, ok);

            end           
        end
    end
end

%% Grafo para imagens nao ligadas diretamente
graph_ = digraph(direct_connections);

% See if there are images that werent connected and connect them
for j=2:numImages
    % If we have a transformation missing from image j to image 1 then
    % calculate it from the other matrices
    if(direct_connections(j, 1) == 0 && vec_ligado_ao_um(j) == 0)  
        P = shortestpath(graph_, j, 1);
        
        Hcurr = eye(4,4);
        for i=1:length(P)-1
            H = cat(2, transforms.R{P(i),P(i+1)}, transforms.T{P(i),P(i+1)});
            H = cat(1, H, [0 0 0 1]);
            
            Hcurr = H * Hcurr;
        end
        
        transforms.R{j,1} = Hcurr(1:3, 1:3);
        transforms.T{j,1} = Hcurr(1:3, 4);
    end
end

%% Run ICP V2 - Bundle adjustment

gridStep = 0.01;
%Initial point cloud
total = xyz{1,1};
cl=reshape(rgbd{1},480*640,3);
%showPointCloud(p1);

for j=2:numImages
    
    %Get the updated point cloud from previous images already bundled
    p1=pointCloud(total,'Color',cl);
    p1 = pcdownsample(p1,'gridAverage',gridStep);
%     figure();
%     showPointCloud(p1);
    
    %Get the points from image j
    xyz_aux = xyz{1,j};
    
    %Get the point cloud object 
    cl2=reshape(rgbd{j},480*640,3);
    p2=pointCloud(xyz_aux,'Color',cl2);
    p2 = pcdownsample(p2,'gridAverage',gridStep);
    %figure();
    %showPointCloud(p2);
    
    %Get the transformation matrix object
    aux = cat(2, transforms.R{j,1}, transforms.T{j,1});
    aux = cat(1, aux, [0 0 0 1]);
    tform1 = affine3d(aux');
    
    %First the moving cloud, second the fixed cloud
    tform2 = pcregrigid(p2, p1, 'Extrapolate',true, 'MaxIterations', 5, 'InitialTransform', tform1, 'InlierRatio', 0.1);
    aux = tform2.T';
    
    %Save the updated transformation
    transforms.R{j,1} = aux(1:3, 1:3);
    transforms.T{j,1} = aux(1:3, 4);
    
    %Update the total pointcloud with new information
    xyz_aux = transforms.R{j,1} * xyz{1,j}' + repmat(transforms.T{j,1},1, length(xyz{1,j}));
    xyz_aux = xyz_aux';
    
    total = cat(1,xyz_aux, total);
    cl = cat(1, cl2, cl);
end

%showPointCloud(p1);


%% Visualizacao total
total = xyz{1,1};
cl=reshape(rgbd{1},480*640,3);
cltotal = cl;

for j=2:numImages
    xyz_aux = transforms.R{j,1} * xyz{1,j}' + repmat(transforms.T{j,1},1, length(xyz{1,j}));
    xyz_aux = xyz_aux';
    
    total = cat(1,xyz_aux, total);
    cl2=reshape(rgbd{j},480*640,3);
    cltotal = cat(1, cl2, cltotal);
    
    %p=pointCloud(total,'Color',cltotal);
    %showPointCloud(p);
    %pause;
end

p=pointCloud(total(),'Color',cltotal());
% Downsampling the results so that matlab does not panic!
p = pcdownsample(p, 'gridAverage', 0.01);
showPointCloud(p);

%% FOR SEVERAL IMAGES mas apenas comparando as adjacentes e nao todas para todas

% Objects - vector of structures where moving objects were identified
objects = [];

%Initial point cloud
total_pjn = [];
cl_pjn = [];

%threshold de numero de pontos a partir do qual considero que os pontos sao
%de facto moveis, caso contrario digo que sao lixo
THRESHOLD = 15000;
k=1;

for n=1:numImages
    for j=n:numImages
        
        obj_moved_n = [];
        obj_moved_n_cor = [];
        obj_moved_j = [];
        obj_moved_j_cor = [];
        
        if(abs(j-n) == 1)
            
            % find the all points that have correspondence in the both images
            % points xyz of second imagem -> xyz{2}
            % aplicar a equa????o R*[xyz_2] + T, para obter os pontos xyz_2 na pc_1
            xyz_trans = transforms.R{j,n} * xyz{j}' + repmat(transforms.T{j,n},1, length(xyz{j}));
            img_jn = [xyz_trans(1,:) ; xyz_trans(1,:) ; zeros(1,640*480)];
            
            % fazer corresponder as cordenadas xyz_trans na imagem de profundidade
            uvz = Depth_cam.K * xyz_trans;
            uvz = [fix(uvz(1,:)./uvz(3,:)) ; fix(uvz(2,:)./uvz(3,:)) ; uvz(3,:)];
            
            % saber os pontos que da imagem 2 nao t??m correspondencia na imagem 1, e
            % para esse pontos colocar o Z a zero para nao ser mapeada na pc
            for index = 1 : length(uvz)
                if(uvz(1,index)<1 || uvz(1,index)>640 || uvz(2,index)<1 || uvz(2,index)>480)
                    uvz(3,index) = 0;
                end
            end
            %obter apenas os pixeis de j que sao vistos de n
            pixeis_uteis = uvz(:,(uvz(3,:)~=0));
            
            % Get the vector of indices to index the pixels in the rgb of
            % image n
            ind_n = sub2ind([480 640],pixeis_uteis(2,:),pixeis_uteis(1,:));
            
            % Get the vector of indices to index the pixels in the rgb of
            % image j
            ind_jn = find(uvz(3,:)~=0);
            
            %pontos de j convertidos nas medidas de n
            common_poits_jn = xyz_trans(:,ind_jn);
            
            %pontos comuns da imgem n
            xyz_aux_n = xyz{n}';
            img_n = [xyz{n}(:,1)' ; xyz{n}(:,2)' ; zeros(1,640*480)];
            common_poits_n = xyz_aux_n(:,ind_n);
            
            %pontos comuns da imgem 2
            xyz_aux_j = xyz{j}';
            img_j = [xyz{j}(:,1)' ; xyz{j}(:,2)' ; zeros(1,640*480)];
            common_poits_j = xyz_aux_j(:,ind_jn);
            
            % buscar a cor respetiva dos pontos comuns
            cor_n = reshape(rgbd{n},480*640,3)';
            cor_j = reshape(rgbd{j},480*640,3)';
            
            % buscar a cor apenas dos pixeis comuns
            cor_n_test = cor_n(:,ind_n);
            cor_j_test = cor_j(:,find(uvz(3,:)~=0));
         
            % SO PARA TESTAR -- NAO SAO PRECISAS ESTA POINTCLO
            
%             pjn = pointCloud(common_poits_jn','Color',cor_j_test');
%             figure();
%             showPointCloud(pjn)

%             pn = pointCloud(common_poits_n','Color',cor_n_test');
%             figure();
%             showPointCloud(pn)

%             pj = pointCloud(common_poits_j','Color',cor_j'); 
%             figure();
%             showPointCloud(pj)

            %subrepoe as duas pcs
%             figure();
%             pcshowpair(pn,pjn)
            
            
            % Detetar objetos em movimentos em diferentes imagens

            %para os pontos que diferem mais de 0.1 verificar qual a imagem que tem o Z
            %menor
            sub_Z = abs(common_poits_jn(3,:) - common_poits_n(3,:));
            
            ind_z_move = find(sub_Z > 0.1);
            
            ind_n_move = ind_n(ind_z_move);
            ind_jn_move = ind_jn(ind_z_move);
            
            % colocar a cordenada z com 1 nos pixeis que foram detetados
            % como moveis
            img_jn(3,ind_jn_move) = 1;
            img_n(3,ind_n_move) = 1;
            img_j(3,ind_jn_move) = 1;
            
            %verifica em que imagens efetivamente esta o obteto movel
            ind_correct_n = ind_n_move(find(xyz_aux_n(3,ind_n_move) > xyz_aux_j(3,ind_jn_move)));
            ind_correct_j = ind_jn_move(find(xyz_aux_j(3,ind_jn_move) > xyz_aux_n(3,ind_n_move)));
            
            % coloca na imagem binaria o que antes esta a 1 para 0 o que
            % significa o pixel que mudou era objeto movel da outra imagem
            % e nao desta
            img_n(3,ind_correct_n) = 0;
            img_j(3,ind_correct_j) = 0;
            img_jn(3,ind_correct_j) = 0;
                    
            %Get the binary image to in order to use bwlabel
            img_jn_r = reshape(img_jn(3,:),[480 640]);
            img_n_r = reshape(img_n(3,:),[480 640]);
            img_j_r = reshape(img_j(3,:),[480 640]);
            
            %remove some noise using a median filter
            img_n_r = medfilt2(img_n_r, [10 10]);
            img_j_r = medfilt2(img_j_r, [10 10]);
            
%             figure()
%             imagesc(img_n_r)
%             figure()
%             imagesc(img_j_r)
%             pause
            
            %Get the individual objects for image n
            [L_n, num_compo_n]= bwlabel(img_n_r);
            
             figure()
             imagesc(L_n)
            
            
            for num_obj = 1 : num_compo_n
                
                object = xyz_aux_n(:,find(L_n(:)' == num_obj));
                object_cor = cor_n(:,find(L_n(:)' == num_obj));
                
                %If we have a good amount of points in that object, it
                %means it is really an object that moved
                if (length(object) > THRESHOLD )
                    [~, numb] = size(object);
                    %dbug
                    imagem_aux = zeros(1, 480 *640);
                    imagem_aux(find(L_n(:)' == num_obj)) = 1;
                    figure()
                    imagesc(reshape(imagem_aux, 480, 640))
                    title(['Numero de pontos ',num2str(numb)])
                    
                    obj_moved_n = object;
                    obj_moved_n_cor = object_cor;
                    
                    %convert to reference frame 1
                    obj_moved_n_1 = transforms.R{n,1} * obj_moved_n + repmat(transforms.T{n,1},1, length(obj_moved_n));
                    
                    %save the object into a cell
                    aux_obj = cell(1,1);
                    aux_struct = struct('xyz', 'framenum');
                    aux_struct.xyz = obj_moved_n_1';
                    aux_struct.framenum = n;
                    aux_obj{1} = aux_struct;
                    
                    %concatenate the cell into the cell array of objects
                    objects = [objects aux_obj];
                    
%                     figure()
%                     p=pointCloud(aux_struct.xyz);
%                     showPointCloud(p)
                                              
                end
            end
            
            %Get the individual objects for image j
            [L_j, num_compo_j]= bwlabel(img_j_r, 4);
            
             %figure()
             %imagesc(L_j)
% %             pause
            
            for num_obj = 1 : num_compo_j
                object = xyz_aux_j(:,find(L_j(:)' == num_obj));
                object_cor = cor_j(:,find(L_j(:)' == num_obj));
                
                
                
                %If we have a good amount of points in that object, it
                %means it is really an object that moved
                if (length(object) > THRESHOLD )
                    [~, numb] = size(object);
                    %dbug
                    imagem_aux = zeros(1, 480 *640);
                    imagem_aux(find(L_n(:)' == num_obj)) = 1;
                    figure()
                    imagesc(reshape(imagem_aux, 480, 640))
                    title(['Numero de pontos ',num2str(numb)])
                    
                    obj_moved_j = object;
                    obj_moved_j_cor = object_cor;
                    
                    %convert to reference frame 1
                    obj_moved_j_1 = transforms.R{j,1} * obj_moved_j + repmat(transforms.T{j,1},1, length(obj_moved_j));
                    
                    %save the object into a cell
                    aux_obj = cell(1,1);
                    aux_struct = struct('xyz', 'framenum');
                    aux_struct.xyz = obj_moved_j_1';
                    aux_struct.framenum = j;
                    aux_obj{1} = aux_struct;
                    
                    %concatenate the cell into the cell array of objects
                    objects = [objects aux_obj];
                    
%                     figure()
%                     p=pointCloud(aux_struct.xyz);
%                     showPointCloud(p)
                                              
                end
            end
            
            % teste
            
            %obj_moved_n = obj_moved_n

            [~,num_points_n] = size(obj_moved_n);

            if(num_points_n ~= 0)
                pn_move = pointCloud(obj_moved_n','Color',obj_moved_n_cor');
                %figure();
                %showPointCloud(pn_move)
                %title(['Moving objects in imagem ',num2str(n)])
            else
                fprintf('N??o existem objetos em movimento na imagem %d \n', n);
            end
            
            [~,num_points_j] = size(obj_moved_j);

            if(num_points_j ~= 0)
                pj_move = pointCloud(obj_moved_j','Color',obj_moved_j_cor');
                %figure();
                %showPointCloud(pj_move)
                %title(['Moving objects in imagem ',num2str(j)])
            else
                fprintf('N??o existem objetos em movimento na imagem %d \n', j);
            end
            
            % mostra os objetos movidos nas varias imagens mas na mesma point cloud com
            % cores diferentes
%             num_points = num_points_n + num_points_j;
% 
%             if(num_points ~= 0)
%                 figure();
%                 pcshowpair(pn_move,pj_move)
%             else
%                 fprintf('N??o existem objetos em movimento entre as imagens %d-%d \n', n,j);
%             end

            aux_total = cat(2, obj_moved_n, obj_moved_j);
            aux_cl = cat(2, obj_moved_n_cor, obj_moved_j_cor);
            
            total_pjn = cat(2,aux_total, total_pjn);
            cl_pjn = cat(2, aux_cl, cl_pjn);
            
            total_pjn = cat(2,aux_total, total_pjn);
            cl_pjn = cat(2, aux_cl, cl_pjn);   
        end
    end
end

% mostra uma pointcloud com todos os objetos moveis
[~,num_points] = size(total_pjn);

if(num_points ~= 0)
    pc_total = pointCloud(total_pjn','Color',cl_pjn');
    figure();
    showPointCloud(pc_total)
else
    disp('N??o existem imagem em movimento');
end

%% Print moving points from the struct

for i = 1 : length(objects)
    if (isempty(objects{i}.xyz) ~= 1)
        figure();
        plot3(objects{i}.xyz(:,1), objects{i}.xyz(:,2), objects{i}.xyz(:,3), '.', 'MarkerSize', 0.5)
        title(['objetos representados na imagem 1 da IMAGEM ',num2str(objects{i}.framenum)])
    end
end