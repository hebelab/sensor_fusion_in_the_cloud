% 3D Lidar point cloud
veloReader = velodyneFileReader("lidar\300\2022-12-10-16-53-10_Velodyne-VLP-16-Data.pcap","VLP16");
lidar_pc = readFrame(veloReader);

% Stereo camera point cloud
camera_pc = pcread("ZED\vga\point_cloud_PLY_3029_376_06-12-2022-19-40-31.ply");
%%
savePointCloud(lidar_pc, 'lidar_pc.mat');
savePointCloud(camera_pc, 'camera_pc.mat');
%%
pcshow(lidar_pc);
pcshow(camera_pc);
%%
lidar_pc_den = pcdenoise(lidar_pc);
camera_pc_den = pcdenoise(camera_pc);
%% 
% The reason that your Location argument is 16xNx3 instead of 16Nx3 is likely 
% because the Velodyne LiDAR sensor captures data in multiple "sweeps" or "scans" 
% of the environment, each with a different vertical field of view (FOV). Each 
% of these sweeps or scans is stored as a separate "slice" or "layer" in the 3D 
% matrix, with each slice representing a different FOV. Therefore, the matrix 
% is 16 layers (sweeps) by N points by 3 dimensions (x,y,z) to reflect the 16 
% different FOVs.

lidar_pts = reshape(lidar_pc_den.Location, [], 3);
%% 
% Camera point cloud is around 90 degree rotated along the x-axis. Hence, I 
% re-rotate to the desired location to match the domains. I use the affine transformation 
% to achieve this.
% 
% 

% Rotate x-axis
a = pi/2;

A = [1 0 0 0;...
     0 cos(a) sin(a) 0;...
     0 -sin(a) cos(a) 0;...
     0 0 0 1];

tform = affine3d(A);

camera_pc_rot = pctransform(camera_pc_den, tform);

% Rotate z-axis
a = pi/20;

A = [cos(a) sin(a) 0 0;...
     -sin(a) cos(a) 0 0;...
     0 0 1 0;...
     0 0 0 1];

tform = affine3d(A);

camera_pc_rot = pctransform(camera_pc_rot, tform);

% Get Nx3 matrix
camera_pts = camera_pc_rot.Location;
%%
% Visualize Lidar point cloud
figure;
scatter3(lidar_pts(:,1), lidar_pts(:,2), lidar_pts(:,3), 'Marker', '.');
title('Lidar Point Cloud');
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
% Visualize camera point cloud
figure;
scatter3(camera_pts(:,1), camera_pts(:,2), camera_pts(:,3), 'Marker', '.');
title('Camera Point Cloud');
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
% Save preprocessed point clouds
savePointCloud(lidar_pts, 'lidar_pts.mat');
savePointCloud(camera_pts, 'camera_pts.mat');
%% Function Definitions

function plyToMat(plyFile, matFile)
    % plyToMat converts a PLY file to a MAT file
    % 
    % Inputs:
    %   plyFile: Name of PLY file to convert (including file path and extension)
    %   matFile: Name of resulting MAT file (including file path and extension)
    
    % Read PLY file
    [pts, faces] = plyread(plyFile);
    
    % Convert faces to face vertex indices
    faces = faces.vertex_indices;
    
    % Save points and faces to MAT file
    save(matFile, 'pts', 'faces');
end

function savePointCloud(pts, fileName)
    % savePointCloud saves a point cloud as an Nx3 matrix
    % 
    % Inputs:
    %   pts: Point cloud data (Nx3 matrix)
    %   fileName: Name of resulting file (including file path and extension)
    
    % Save point cloud to file
    save(fileName, 'pts');
end