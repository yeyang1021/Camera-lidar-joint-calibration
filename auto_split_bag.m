% clc;
% clear all;

%% 读取bag包
filePath = '2020-11-06-11-05-23.bag';
bag_all = rosbag(filePath);

%% 设置标定板属性 
Checkerboard_vertical = 4; % 对应棋盘格需要检测的角点个数，5个棋盘格有4个角点
Checkerboard_horizontal = 6; % 
Checkerboard_detection_size = Checkerboard_vertical * Checkerboard_horizontal;

%% 读取bag包中图像
camera_ = select(bag_all, 'Topic', '/camera/image/compressed');
message=select(camera_,'MessageType', 'sensor_msgs/CompressedImage');
data=readMessages(message);
images = cell(1,size(data,1));
for i=1:size(data,1)
   data_each = data{i,1}; 
   image = readImage(data_each);   
   images{1,i} = image;
end


%% 检测图像中的棋盘格 
imagePointsList = zeros(size(data,1), 24, 2);
for i=1:size(data,1)
    image = images{1,i};
    [imagePointsLists, boardSize, imagesUsed] = detectCheckerboardPoints(image, 'MinCornerMetric', MinCornerMetric);  
    if size(imagePointsLists,1) ~= Checkerboard_detection_size
        imagePointsList(i,:,:) = -1;
    else
        imagePointsList(i,:,:) = imagePointsLists;
    end
    
end

value = abs(imagePointsList(1:end-1,:,:) - imagePointsList(2:end,:,:));
values = sum(value,2);
values_ = sum(values,3);

%% 找峰值，获取关键帧
[pks,locs] = findpeaks(-values_, 'MinPeakWidth',10);  
pks1 = pks>-0.5;
pks_ = pks(pks1);
locs_ = locs(pks1);


%% 找到对应的激光雷达数据
lidar_ = select(bag_all, 'Topic', '/rslidar_points');
message_lidar=select(lidar_,'MessageType','sensor_msgs/PointCloud2');
data_lidar=readMessages(message_lidar);

rate = size(data_lidar,1) / size(data,1);
locs_lidar = int32(locs_ * rate); 

lidars = cell(1,size(locs_,1));
count = 1;
for i=1:size(data_lidar,1) 
   if ismember(i,locs_lidar) 
       data_each = data_lidar{i,1}; 
       lidar = readXYZ(data_each);   
       lidars{1,count} = lidar;
       savename = [num2str(i), '.pcd'];
       pcwrite(pointCloud(lidar), savename, 'Encoding', 'ascii');
       count=count+1;
   end
end



