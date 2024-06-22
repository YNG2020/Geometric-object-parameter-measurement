load 'cleanData.mat' cleanData
load 'RATIO.mat' RATIO;
rng(1);
close all
%% 检查数据
for i = [25 26 27 28]
    data = cleanData{i};
    
    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象
    
    % 可视化结果
    figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");
    
    % 原始点云数据
    pcshow(ptCloud, 'MarkerSize', 40);
    title('原始点云');
    xlabel('X'); ylabel('Y'); zlabel('Z');
end
close all

%% 构造分离平面，分开圆柱顶面与其它点云
data = cleanData{26};

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
subplot(1, 3, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

xlim = [0.1 0.15];
ylim = [0.05 0.1];
zlim = [0.05 0.1];

p1 = [0.105109870433807,0.071799241006374,0.086261332035065];
p2 = [0.112238720059395,0.086967818439007,0.082276225090027];
p3 = [0.132097870111465,0.080169595777988,0.073791086673737];

[circle, ~] = divideData(data, p1, p2, p3, xlim, ylim, zlim);

%% 使用RANSAC方法去噪
maxDistance = 0.005; % 平面距离的阈值

ptCloud = pointCloud(circle);
[model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99.99, 'MaxNumTrials', 10000);

% 分离平面点和平面外点
other = circle(remainIdx, :);
cleanCircle = circle(inlierIdx, :); % 平面上的点

subplot(1, 3, 3);
hold on
title('去噪结果');
pcshow(cleanCircle, 'g', 'MarkerSize', 40);
pcshow(other, 'r', 'MarkerSize', 40);

%% 使用PCA方法降维 + 使用最小覆盖圆求取直径
[PCACircle] = myPCA(cleanCircle);

[center, radius] = findMinCircle(PCACircle);
diameter = 2 * radius;

% 绘制结果
figure;
scatter(PCACircle(:,1), PCACircle(:,2), 'filled');
hold on;
theta = linspace(0, 2*pi, 100);
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
plot(x, y, 'r-', 'LineWidth', 2);
plot(center(1), center(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
title('最小覆盖圆');
axis equal;
hold off;

%% 输出结果
fprintf('The diameter is: %f\n', RATIO * diameter);

%% 辅助函数：用于构造分割平面方程数据分离
function [inlier, outliner] = divideData(data, p1, p2, p3, xlim, ylim, zlim)
    [a, b, c, d] = constructPlaneEquation(p1, p2, p3);
    % 将数据分为两块
    inlierIdx = zeros(size(data, 1), 1); outlierIdxIdx = zeros(size(data, 1), 1);
    for i = 1 : size(data, 1)
        p1 = data(i, 1);
        p2 = data(i, 2);
        p3 = data(i, 3);
        if a * p1 + b * p2 + c * p3 + d < 0
            inlierIdx(i) = 1;
        else
            outlierIdxIdx(i) = 1;
        end
    end

    inlier = data(inlierIdx == 1, :);
    outliner = data(outlierIdxIdx == 1, :);
    
    % 分离点云数据
    subplot(1, 3, 2);
    pcshow(outliner, 'r', 'MarkerSize', 40);
    hold on
    plotPlane(a, b, c, d, xlim, ylim, zlim, 'y')
    pcshow(inlier, 'g', 'MarkerSize', 40);
    title('分离点云数据');
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

function [a, b, c, d] = constructPlaneEquation(P1, P2, P3)
    % 输入：三个点，P1, P2, P3，格式为 [x, y, z]
    % 输出：平面方程的系数 a, b, c, d

    % 计算两个向量
    v1 = P2 - P1;
    v2 = P3 - P1;
    
    % 计算法向量（叉积）
    normal = cross(v1, v2);
    
    % 提取法向量的分量
    a = normal(1);
    b = normal(2);
    c = normal(3);
    
    % 计算常数项 d
    d = -(a * P1(1) + b * P1(2) + c * P1(3));
end

function plotPlane(a, b, c, d, xlim, ylim, zlim, color)
    % a, b, c, d 为平面方程 ax + by + cz + d = 0 的系数
   
    x = zeros(4, 1);
    x(1) = -(b * ylim(1) + c * zlim(1) + d) / a;x(2) = -(b * ylim(1) + c * zlim(2) + d) / a;
    x(3) = -(b * ylim(2) + c * zlim(1) + d) / a;x(4) = -(b * ylim(2) + c * zlim(2) + d) / a;
    [maxDiffX] = findMaxDiff(x);
    y = zeros(4, 1);
    y(1) = -(a * xlim(1) + c * zlim(1) + d) / b;y(2) = -(a * xlim(1) + c * zlim(2) + d) / b;
    y(3) = -(a * xlim(2) + c * zlim(1) + d) / b;y(4) = -(a * xlim(2) + c * zlim(2) + d) / b;
    [maxDiffY] = findMaxDiff(y);
    z = zeros(4, 1);
    z(1) = -(a * xlim(1) + b * ylim(1) + d) / c;z(2) = -(a * xlim(1) + b * ylim(2) + d) / c;
    z(3) = -(a * xlim(2) + b * ylim(1) + d) / c;z(4) = -(a * xlim(2) + b * ylim(2) + d) / c;
    [maxDiffZ] = findMaxDiff(z);

    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffX
        [Y, Z] = meshgrid(linspace(ylim(1), ylim(2)), linspace(zlim(1), zlim(2)));
        X = -(b * Y + c * Z + d) / a;
    end
    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffY
        [X, Z] = meshgrid(linspace(xlim(1), xlim(2)), linspace(zlim(1), zlim(2)));
        Y = -(a * X + c * Z + d) / b;
    end
    if min([maxDiffX maxDiffY maxDiffZ]) == maxDiffZ
        [X, Y] = meshgrid(linspace(xlim(1), xlim(2)), linspace(ylim(1), ylim(2)));
        Z = -(a * X + b * Y + d) / c;
    end
    % 绘制平面
    surf(X, Y, Z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', color);

end

function [maxDiff] = findMaxDiff(x)
    % 计算所有可能的差值组合
    comb_indices = nchoosek(1:length(x), 2);
    differences = abs(x(comb_indices(:,1)) - x(comb_indices(:,2)));
    % 找出最大的差值
    maxDiff = max(differences);
end

%% 辅助函数：PCA
function [projected_points] = myPCA(points)
    % 中心化数据
    mean_points = mean(points);
    centered_points = points - mean_points;
    
    % 计算协方差矩阵
    cov_matrix = cov(centered_points);
    
    % 求解特征值和特征向量
    [V, D] = eig(cov_matrix);
    
    % 按特征值大小排序
    [~, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    projected_points = centered_points * V(:, 1:2);
end

%% 辅助函数：最小覆盖圆
function [center, radius] = findMinCircle(points)
    % 主函数，用于找到点集的最小覆盖圆
    % points: 点集（nx2矩阵）
    % center: 最小覆盖圆的中心
    % radius: 最小覆盖圆的半径
    
    % 调用递归辅助函数
    [center, radius] = minCircleHelper(points, []);
end

function [center, radius] = minCircleHelper(P, R)
    % 递归辅助函数，P是剩余点集，R是边界上的点
    if isempty(P) || numel(R) == 3
        % 基本情况：如果没有点或边界上有三个点
        [center, radius] = getCircle(R);
        return;
    end
    
    % 随机选择一个点进行递归
    P = P(randperm(size(P, 1)), :); % 打乱点集顺序
    p = P(end, :); % 选择最后一个点
    P(end, :) = []; % 从点集P中移除这个点
    
    [center, radius] = minCircleHelper(P, R);
    
    % 检查这个点是否在当前最小圆内
    if norm(p - center) > radius
        % 如果不在，则将这个点添加到边界点集R
        [center, radius] = minCircleHelper(P, [R; p]);
    end
end

function [center, radius] = getCircle(R)
    % 计算点集R的最小覆盖圆
    switch size(R, 1)
        case 0
            center = [0, 0];
            radius = 0;
        case 1
            center = R(1, :);
            radius = 0;
        case 2
            % 两点的情况
            center = (R(1, :) + R(2, :)) / 2;
            radius = norm(R(1, :) - R(2, :)) / 2;
        case 3
            % 三点的情况，计算经过三点的圆
            A = R(1, :);
            B = R(2, :);
            C = R(3, :);
            D = 2 * (A(1) * (B(2) - C(2)) + B(1) * (C(2) - A(2)) + C(1) * (A(2) - B(2)));
            if D == 0
                center = [NaN, NaN];
                radius = NaN;
            else
                center = [(norm(A)^2 * (B(2) - C(2)) + norm(B)^2 * (C(2) - A(2)) + norm(C)^2 * (A(2) - B(2))) / D, ...
                          (norm(A)^2 * (C(1) - B(1)) + norm(B)^2 * (A(1) - C(1)) + norm(C)^2 * (B(1) - A(1))) / D];
                radius = norm(A - center);
            end
        otherwise
            error('Unexpected number of boundary points.');
    end
end