# 测量3D点云数据中的几何体尺寸

本报告记录了本人使用3D数据常用的处理算法，对用3D点云数据代表的几何体进行尺寸测量。

## 1 引言

### 1.1 3D点云

3D点云数据是由大量的点组成的集合，每个点在空间中都有一个三维坐标$(X, Y, Z)$。这些点通常用于描述物体的表面或环境的形状和结构。点云数据广泛应用于计算机视觉、三维建模、地图测绘、建筑设计等领域。

![3D Laser Scanning Point](figure/f01.png "3D Laser Scanning Point")

### 1.2 要进行几何尺寸测量的物体

一共有7个点云测量任务，包含9个不同的物体。每个任务都有来自不同视角拍摄的4个点云数据集。原始数据集格式为.npz，为在MATLAB上操作，已将它们全都转换为.mat文件。

![object1](figure/f02.png "object1"){height=174} ![object2](figure/f03.png "object2"){height=174} ![object3](figure/f04.png "object3"){height=174} ![object4](figure/f05.png "object4"){height=174} ![object5](figure/f06.png "object5"){height=174} ![object6](figure/f07.png "object6"){height=174} ![object7](figure/f08.png "object7"){height=174} ![object8](figure/f09.png "object8"){height=174} ![object9](figure/f10.png "object9"){height=174}

在该工作中值得注意的是：

- 第5个任务中要被测量的两个物体叠放在一起。
- 第6个任务中的点云数据集中同时存在两个物体。
- 第7个任务中要被测量的物体是手持物体拍摄的。

如下图：
![case1](figure/f11.png "case1"){height=174} ![case2](figure/f12.png "case2"){height=174} ![case3](figure/f13.png "case3"){height=174}

## 2 算法介绍

本节将对在该工作中所使用的算法进行介绍。

### 2.1 RANSAC

随机抽样一致（RANdom SAmple Consensus，RANSAC）算法。它采用迭代的方式从一组包含离群的被观测数据中估算出数学模型的参数。 RANSAC是一个非确定性算法，在某种意义上说，它会产生一个在一定概率下合理的结果，而更多次的迭代会使这一概率增加。此RANSAC算法在1981年由Fischler和Bolles首次提出。

RANSAC的基本假设是：

1. “内群”(inlier, 似乎译为内点群更加妥当，即正常数据，正确数据)数据可以通过几组模型的参数来叙述其分布，而“离群”(outlier,似乎译为外点群更加妥当，异常数​​据)数据则是不适合模型化的数据。
2. 数据会受噪声影响，噪声指的是离群，例如从极端的噪声或错误解释有关数据的测量或不正确的假设。
3. RANSAC假定，给定一组（通常很小）的内点群，存在一个程序，这个程序可以估算最佳解释或最适用于这一数据模型的参数。

#### 2.1.1 范例

这里用一个简单的例子来说明，在一组数据点中找到一条最适合的线。假设，此有一组集合包含了内点群以及外点群，其中内点群包含可以被拟合到线段上的点，而外点群则是无法被拟合的点。如果我们用简单的最小二乘法来找此线，我们将无法得到一条适合于内点群的直线，因为最小二乘法会受外点群影响而影响其结果。而用RANSAC，可以只由内点群来计算出模型，而且概率还够高。然而，RANSAC无法保证结果一定最好，所以必须小心选择参数，使其能有足够的概率得到正确的拟合直线。

![alt text](figure/f14.png "RANSAC例子")
