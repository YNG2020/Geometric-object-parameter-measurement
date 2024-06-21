load 'cleanData.mat' cleanData
rng(1);

%% 检查数据
data = [cleanData{13}; cleanData{14}; cleanData{15}; cleanData{16}];

% 使用 MATLAB 内置的 pcfitplane 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% 将点云最高点视为球体最高点
diameter = 0;
for i = [13 14 15 16]
    data = cleanData{i};
    diameter = diameter + max(data(:, 3));
end

%% 输出结果
load RATIO.mat RATIO
diameter = diameter / 4;
fprintf("The radius is: %f\n", diameter * RATIO / 2);
