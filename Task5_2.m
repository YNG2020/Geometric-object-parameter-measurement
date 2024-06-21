load 'cleanData.mat' cleanData
load 'RATIO.mat' RATIO;
rng(1);
close all
%% 分离出长方形块
data = cleanData{20};

% 使用 MATLAB 内置的 pcfitplane 函数
ptCloud = pointCloud(data); % 将数据转换为点云对象

% 使用 RANSAC 算法检测平面
maxDistance = 0.005; % 平面距离的阈值
[model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, [0 0 1], 'Confidence', 99.999, 'MaxNumTrials', 10000);

% 分离平面点和平面外点
planePoints = data(inlierIdx, :); % 平面上的点
remainingPoints = data(remainIdx, :); % 平面外的点

% 可视化结果
figureX = figure('units','normalized','outerposition', [0 0 1 1], 'Name', "data");

% 原始点云数据
subplot(1, 3, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 以下通过取定点，粗糙地求得四个平面的法向量
p11 = [0.025308936834335,-0.012671858072281,0.029289424419403];
p12 = [-0.002643644809723,-0.019143104553223,0.031945168972015];
p13 = [0.012647002935410,-0.012847259640694,0.037336528301239];
normalVector1 = calNormalVector(p11, p12, p13);

p21 = [0.029100000000000,0.022923288445946,0.042700000000000];
p22 = [-0.007904857397079,0.004141584038734,0.033487319946289];
p23 = [0.011776685714722,0.012107275426388,0.030666410923004];
normalVector2 = calNormalVector(p21, p22, p23);

p31 = [0.027829915285110,-0.004392862319946,0.033986330032349];
p32 = [0.015251308679581,4.596039652824402e-04,0.053958117961884];
p33 = [0.018285453319550,0.007391870021820,0.038425326347351];
normalVector3 = calNormalVector(p31, p32, p33);

p41 = [0.010593384504318,0.008413165807724,0.023686230182648];
p42 = [0.009290814399719,-0.018893346190453,0.024638593196869];
p43 = [-0.003770887851715,-0.024375110864639,0.020918548107147];
normalVector4 = calNormalVector(p41, p42, p43);

normalVector = zeros(4, 3);
normalVector(1, :) = normalVector1;
normalVector(2, :) = normalVector2;
normalVector(3, :) = normalVector3;
normalVector(4, :) = normalVector4;

% 构建分离平面
[a, b, c, d] = constructPlaneEquation(p21, p22, p23);

% 分离长方体与三棱柱
inlierIdx = zeros(size(data, 1), 1); outlierIdxIdx = zeros(size(data, 1), 1);
for i = 1 : size(data, 1)
    p1 = data(i, 1);
    p2 = data(i, 2);
    p3 = data(i, 3);
    if a * p1 + b * p2 + c * p3 + d >= 0
        inlierIdx(i) = 1;
    else
        outlierIdxIdx(i) = 1;
    end
end

inlierIdx = find(inlierIdx);
outlierIdxIdx = find(outlierIdxIdx);

cuboid = data(inlierIdx, :);
Tri_Prism = data(outlierIdxIdx, :);

% 分离点云数据
subplot(1, 3, 2);
pcshow(cuboid, 'g', 'MarkerSize', 40);
hold on
plotPlane(a, b, c, d)
pcshow(Tri_Prism, 'r', 'MarkerSize', 40);
title('分离点云数据');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% 精细地求得五个平面的法向量
A = zeros(5, 3);
b = zeros(5, 1);
ptCloud = pointCloud(cuboid); % 将数据转换为点云对象
% 使用 RANSAC 算法检测平面
maxDistance = 0.001; % 平面距离的阈值
[model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, 'Confidence', 99, 'MaxNumTrials', 10000);

A(5, :) = model.Normal;
b(5) = -model.Parameters(4);

% 分离平面点和平面外点
planePoints = cuboid(inlierIdx, :); % 平面上的点

subplot(1, 3, 3);
hold on
pcshow(planePoints, 'y', 'MarkerSize', 40);
color = ['r', 'g', 'b', 'm'];
for i = 1 : 4
    
    % 使用 RANSAC 算法检测平面
    maxDistance = 0.001; % 平面距离的阈值
    [model, inlierIdx, remainIdx] = pcfitplane(ptCloud, maxDistance, normalVector(i, :), 5, 'Confidence', 99, 'MaxNumTrials', 10000);

    A(i, :) = model.Normal;
    b(i) = -model.Parameters(4);

    % 分离平面点和平面外点
    planePoints = cuboid(inlierIdx, :); % 平面上的点
    hold on
    pcshow(planePoints, color(i), 'MarkerSize', 40);

end

%% 通过5个平面相互之间的交点确定出三棱柱的形状

A1 = [A(1, :); A(3, :); A(5, :)];
b1 = [b(1); b(3); b(5)];
x1 = A1 \ b1;

A2 = [A(1, :); A(4, :); A(5, :)];
b2 = [b(1); b(4); b(5)];
x2 = A2 \ b2;

A3 = [A(1, :); A(3, :); A(4, :)];
b3 = [b(1); b(3); b(4)];
x3 = A3 \ b3;   

A4 = [A(2, :); A(3, :); A(5, :)];
b4 = [b(2); b(3); b(5)];
x4 = A4 \ b4;

A5 = [A(2, :); A(3, :); A(4, :)];
b5 = [b(2); b(3); b(4)];
x5 = A5 \ b5;

A6 = [A(2, :); A(4, :); A(5, :)];
b6 = [b(2); b(4); b(5)];
x6 = A6 \ b6;

len_A = (sqrt((x1 - x2)' * (x1 - x2)) + sqrt((x4 - x5)' * (x4 - x5))) / 2;
len_B = (sqrt((x1 - x3)' * (x1 - x3)) + sqrt((x4 - x6)' * (x4 - x6))) / 2;
len_C = (sqrt((x2 - x3)' * (x2 - x3)) + sqrt((x5 - x6)' * (x5 - x6))) / 2;
len_D = sqrt((x1 - x4)' * (x1 - x4));
%% 输出数据
fprintf('The length of A is: %f\n', RATIO * len_A);
fprintf('The length of B is: %f\n', RATIO * len_B);
fprintf('The length of C is: %f\n', RATIO * len_C);
fprintf('The length of D is: %f\n', RATIO * len_D);

%% 辅助函数
function [normalVector] = calNormalVector(p1, p2, p3)
    v1 = p2 - p1;v2 = p3 - p1;
    normalVector = cross(v1, v2) / norm(cross(v1, v2));
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

function plotPlane(a, b, c, d)
    % a, b, c, d 为平面方程 ax + by + cz + d = 0 的系数
    
    % 创建 x, y 的网格范围
    [X, Z] = meshgrid(-0.025:0.0001:0.04, 0.01:0.0001:0.05);
    
    % 计算 z 的值

    Y = -(a * X + c * Z + d) / b;
    
    % 绘制平面
    surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'y');

end