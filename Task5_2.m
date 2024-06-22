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

xlim = [-0.025 0.04];
ylim = [-0.03 0.03];
zlim = [0.01 0.055];

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

p41 = [0.025308936834335,-0.012671858072281,0.029289424419403];
p42 = [-0.016464646464646,-0.005784523673356,0.015656565656566];
p43 = [0.019673807546496,0.015454545454545,0.020505050505051];
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
plotPlane(a, b, c, d, xlim, ylim, zlim, 'y')
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
plotPlane(model.Parameters(1), model.Parameters(2), model.Parameters(3), model.Parameters(4), xlim, ylim, zlim, 'y')
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
    plotPlane(model.Parameters(1), model.Parameters(2), model.Parameters(3), model.Parameters(4), xlim, ylim, zlim, color(i))
end

%% 通过5个平面相互之间的交点确定出三棱柱的形状

A1 = [A(1, :); A(3, :); A(5, :)];
b1 = [b(1); b(3); b(5)];
p1 = A1 \ b1;

A2 = [A(1, :); A(4, :); A(5, :)];
b2 = [b(1); b(4); b(5)];
p2 = A2 \ b2;

A3 = [A(1, :); A(3, :); A(4, :)];
b3 = [b(1); b(3); b(4)];
p3 = A3 \ b3;   

A4 = [A(2, :); A(3, :); A(5, :)];
b4 = [b(2); b(3); b(5)];
p4 = A4 \ b4;

A5 = [A(2, :); A(4, :); A(5, :)];
b5 = [b(2); b(4); b(5)];
p5 = A5 \ b5;

A6 = [A(2, :); A(3, :); A(4, :)];
b6 = [b(2); b(3); b(4)];
p6 = A6 \ b6;

fprintf('len_A: %f, %f\n', sqrt((p1 - p2)' * (p1 - p2)), sqrt((p4 - p5)' * (p4 - p5)))
fprintf('len_B: %f, %f\n', sqrt((p1 - p3)' * (p1 - p3)), sqrt((p4 - p6)' * (p4 - p6)))
fprintf('len_C: %f, %f\n', sqrt((p2 - p3)' * (p2 - p3)), sqrt((p5 - p6)' * (p5 - p6)))
fprintf('len_D: %f, %f, %f\n', sqrt((p1 - p4)' * (p1 - p4)), sqrt((p2 - p5)' * (p2 - p5)), sqrt((p3 - p6)' * (p3 - p6)))

P = [p1 p2 p3 p4 p5 p6];
for i = 1 : 6
    scatter3(P(1, i), P(2, i), P(3, i), 'filled', 'w')
end



len_A = (sqrt((p2 - p3)' * (p2 - p3)) + sqrt((p5 - p6)' * (p5 - p6))) / 2;
len_B = (sqrt((p1 - p3)' * (p1 - p3)) + sqrt((p4 - p6)' * (p4 - p6))) / 2;
len_C = (sqrt((p1 - p2)' * (p1 - p2)) + sqrt((p4 - p5)' * (p4 - p5))) / 2;
len_D = sqrt((p1 - p4)' * (p1 - p4));
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