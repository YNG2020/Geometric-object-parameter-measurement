% 1. 读取数据
load 'allData.mat' allData

rng(1);
close all
cleanData = cell(1, 28);
% 以下是数据预处理
for i = [9 10 11 12]
    data = allData{i};
    
    % 2. 使用RANSAC算法检测平面
    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    
    % 3. 使用 RANSAC 算法检测平面
    maxDistance = 0.01; % 平面距离的阈值
    [model, ~, ~] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.999, 'MaxNumTrials', 10000);
    
    % 4. 分离平面点和平面外点
    a = model.Parameters(1);
    b = model.Parameters(2);
    c = model.Parameters(3);
    d = model.Parameters(4);
    
    % 计算到平面的距离
    distances = (a*data(:,1) + b*data(:,2) + c*data(:,3) + d) / sqrt(a^2 + b^2 + c^2);
    
    % 计算目标数据与点云平面的相对位置
    countDir = 0;
    for j = 1 : size(data, 1)
        if abs(distances(j)) > maxDistance
            countDir = countDir + distances(j);
        end
    end
    % 将不位于目标数据一侧的点云数据直接归类到平面上
    for j = 1 : size(data, 1)
        if distances(j) * countDir < 0
            distances(j) = 0.9 * maxDistance;
        end
    end

    % 法向量
    normalVector = model.Normal;
    % 目标水平面法向量
    targetVector = [0, 0, 1];
    
    % 计算旋转轴（法向量和目标向量的叉积）
    rotationAxis = cross(normalVector, targetVector);
    rotationAxis = rotationAxis / norm(rotationAxis); % 单位化
    
    % 计算旋转角度（法向量和目标向量的夹角）
    theta = acos(dot(normalVector, targetVector) / (norm(normalVector) * norm(targetVector)));

    % 使用Rodrigues' 旋转公式构建旋转矩阵
    K = [0 -rotationAxis(3) rotationAxis(2);
         rotationAxis(3) 0 -rotationAxis(1);
        -rotationAxis(2) rotationAxis(1) 0];
    I = eye(3);
    R = I + sin(theta) * K + (1 - cos(theta)) * K^2; % 旋转矩阵
    
    % 应用旋转矩阵，把数据旋转至水平方向
    data = (R * data')'; 
    
    % 分离平面数据与目标数据
    planePoints = data(abs(distances) < maxDistance, :); % 平面上的点
    remainingPoints = data(abs(distances) > maxDistance, :); % 剩余的点（不在平面上）

    remainingPoints(:, 3) = remainingPoints(:, 3) - mean(planePoints(:, 3));  % 将数据挪到z = 0平面上
    if mean(remainingPoints(:, 3) < 0)
        remainingPoints(:, 3) = -remainingPoints(:, 3);
    end
    remainingPoints(:, 2) = remainingPoints(:, 2) - mean(remainingPoints(:, 2)); % 将数据归于平面的原点
    remainingPoints(:, 1) = remainingPoints(:, 1) - mean(remainingPoints(:, 1)); % 将数据归于平面的原点
    cleanData{1, i} = remainingPoints;

    planePoints(:, 3) = planePoints(:, 3) - mean(planePoints(:, 3));  % 将数据挪到z = 0平面上
    planePoints(:, 2) = planePoints(:, 2) - mean(planePoints(:, 2)); % 将数据归于平面的原点
    planePoints(:, 1) = planePoints(:, 1) - mean(planePoints(:, 1)); % 将数据归于平面的原点

    % 5. 可视化结果
    figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data" + num2str(i-1));

    % 原始点云数据
    subplot(1, 3, 1);
    pcshow(ptCloud, 'MarkerSize', 40);
    title('原始点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % 检测到的平面点
    subplot(1, 3, 2);
    pcshow(planePoints, 'r', 'MarkerSize', 40);
    title('检测到的平面点（旋转至水平面）');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % 去除了平面的点云数据
    subplot(1, 3, 3);
    pcshow(remainingPoints, 'b', 'MarkerSize', 40);
    title('去平面的点云（旋转至水平面）');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    savefig("fig" + num2str(i-1) + ".fig");
end

save 'cleanData.mat' cleanData;