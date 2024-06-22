# 测量3D点云数据中的几何体尺寸

本报告记录了本人使用3D数据常用的处理算法，对用3D点云数据代表的几何体进行尺寸测量。

## 引言

### 3D点云

3D点云数据是由大量的点组成的集合，每个点在空间中都有一个三维坐标$(X, Y, Z)$。这些点通常用于描述物体的表面或环境的形状和结构。点云数据广泛应用于计算机视觉、三维建模、地图测绘、建筑设计等领域。

![3D Laser Scanning Point](figure/f01.png "3D Laser Scanning Point")

### 要进行几何尺寸测量的物体

一共有7个点云测量任务，包含9个不同的物体。每个任务都有来自不同视角拍摄的4个点云数据集。原始数据集格式为.npz，为在MATLAB上操作，已将它们全都转换为.mat文件。

![object1](figure/f02.png "object1"){height=174} ![object2](figure/f03.png "object2"){height=174} ![object3](figure/f04.png "object3"){height=174} ![object4](figure/f05.png "object4"){height=174} ![object5](figure/f06.png "object5"){height=174} ![object6](figure/f07.png "object6"){height=174} ![object7](figure/f08.png "object7"){height=174} ![object8](figure/f09.png "object8"){height=174} ![object9](figure/f10.png "object9"){height=174}

在该工作中值得注意的是：

- 第5个任务中要被测量的两个物体叠放在一起；
- 第6个任务中的点云数据集中同时存在两个物体；
- 第7个任务中要被测量的物体是手持物体拍摄的；

如下图：
![case1](figure/f11.png "case1"){height=174} ![case2](figure/f12.png "case2"){height=174} ![case3](figure/f13.png "case3"){height=174}
