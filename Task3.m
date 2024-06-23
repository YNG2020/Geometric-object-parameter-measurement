close all
clear
load data/RATIO.mat RATIO
load data/cleanData.mat cleanData
rng(1);
%% 求三个平面的法向量

% 以下通过取定点，粗糙地求得三个平面的法向量
normalVector = zeros(3, 3);
p1 = [-0.007545530796051,0.002167575061321,0.036722898483276];p2 = [0.010855644941330,-0.001181031111628,0.010518193244934];p3 = [0.010650515556335,0.009665317833424,0.010690510272980];
v1 = p2 - p1;v2 = p3 - p1;
normalVector(1, :) = cross(v1, v2) / norm(cross(v1, v2));
p1 = [-0.007642835378647,-0.001010239124298,0.036338150501251];p2 = [-0.009978532791138,-0.011613370850682,0.014657139778137];p3 = [-0.003675162792206,-0.011406060308218,0.014989793300629];
v1 = p2 - p1;v2 = p3 - p1;
normalVector(2, :) = cross(v1, v2) / norm(cross(v1, v2));
p1 = [-0.007448226213455,0.005345396231860,0.037107706069946];p2 = [-0.013005137443542,0.009109292179346,0.014855325222015];p3 = [0.005410224199295,0.011526970192790,0.017515182495117];
v1 = p2 - p1;v2 = p3 - p1;
normalVector(3, :) = cross(v1, v2) / norm(cross(v1, v2));

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");
data = [cleanData{11}];
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 原始点云数据
subplot(1, 3, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云', 'FontSize', 15);
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);

% 检测到的平面点
subplot(1, 3, 2);
title('检测到的三个平面上的点', 'FontSize', 15);
xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
hold on

color = ['r', 'g', 'b'];
A = zeros(3, 3);
b = zeros(3, 1);
% 精细地求得三个平面的法向量
for i = 1 : 3
    
    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象

    % 使用 RANSAC 算法检测平面
    maxDistance = 0.0005; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, normalVector(i, :), 1, 'Confidence', 99, 'MaxNumTrials', 10000);

    A(i, :) = model.Normal;
    b(i) = -model.Parameters(4);

    % 分离平面点和平面外点
    planePoints = data(inlierIdx, :); % 平面上的点
    remainingPoints = data(remainIdx, :); % 平面外的点

    pcshow(planePoints, color(i), 'MarkerSize', 40);

end

%% 求边长

xlim = [-0.02 0.01];
ylim = [-0.015 0.015];
zlim = [0.01 0.04];
color = ['r', 'g', 'b', 'm'];
subplot(1, 3, 3);
hold on
for i = 1 : 3
    plotPlane(A(i, 1), A(i, 2), A(i, 3), -b(i), xlim, ylim, zlim, color(i));
    % 使用 MATLAB 内置的 pcfitplane 函数
    ptCloud = pointCloud(data); % 将数据转换为点云对象

    % 使用 RANSAC 算法检测平面
    maxDistance = 0.0005; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, normalVector(i, :), 1, 'Confidence', 99, 'MaxNumTrials', 10000);

    A(i, :) = model.Normal;
    b(i) = -model.Parameters(4);

    % 分离平面点和平面外点
    planePoints = data(inlierIdx, :); % 平面上的点
    remainingPoints = data(remainIdx, :); % 平面外的点

    pcshow(planePoints, color(i), 'MarkerSize', 40);
end

plotPlane(0, 0, 1, 0, xlim, ylim, zlim, color(4));

normalVector4 = [0 0 1];
A1 = A; A1(3, :) = normalVector4;
b1 = b; b1(3) = 0;
x1 = A1 \ b1;

A2 = A; A2(2, :) = normalVector4;
b2 = b; b2(2) = 0;
x2 = A2 \ b2;

length = sqrt((x1 - x2)' * (x1 - x2)); 

%% 求角度方法1（交线法）
% 求3个平面的2条交线
n1 = cross(A(1, :), A(2, :));
n2 = cross(A(1, :), A(3, :));

% 求这2条交线的夹角
dot_product = dot(n1, n2);
norm_n1 = norm(n1);
norm_n2 = norm(n2);
cos_theta = dot_product / (norm_n1 * norm_n2);
theta_radians = acos(cos_theta);
deg1 = rad2deg(theta_radians);

%% 求角度方法2（三角函数法）
x = A \ b;
h = RATIO * x(3);
lengthHalf = length * RATIO / 2;
l = sqrt(h^2 + lengthHalf^2 + lengthHalf^2);
deg2 = 2*rad2deg(sinh(lengthHalf / l));

P = [x x1 x2];
for i = 1 : 3
    scatter3(P(1, i), P(2, i), P(3, i), 'filled', 'w')
end
%% 输出结果
fprintf("The degree of A calculated by trigonometric function is: %f°\n", deg1);
fprintf("The degree of A calculated by intersection line is: %f°\n", deg2);
fprintf("The length of B is: %f mm\n", length * RATIO);

%% 辅助函数
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
    title('平面的交线与交点', 'FontSize', 15);
    xlabel('X', 'FontSize', 13); ylabel('Y', 'FontSize', 13); zlabel('Z', 'FontSize', 13);
end

function [maxDiff] = findMaxDiff(x)
    % 计算所有可能的差值组合
    comb_indices = nchoosek(1:length(x), 2);
    differences = abs(x(comb_indices(:,1)) - x(comb_indices(:,2)));
    % 找出最大的差值
    maxDiff = max(differences);
end