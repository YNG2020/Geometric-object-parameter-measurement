load 'cleanData.mat' cleanData
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

% 原始点云数据
subplot(1, 2, 1);
pcshow(ptCloud, 'MarkerSize', 40);
title('原始点云');
xlabel('X'); ylabel('Y'); zlabel('Z');

% 检测到的平面点
subplot(1, 2, 2);
title('检测到的平面点');
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on

color = ['r', 'g', 'b'];
A = zeros(3, 3);
b = zeros(3, 1);
data = [cleanData{11}];
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

load RATIO.mat RATIO

%% 求边长
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

%% 输出结果
fprintf("The degree of A is: %f\n", deg2);
fprintf("The length of B is: %f\n", length * RATIO);


