# detection_evaluation

## 介绍

该文件夹放置的是用于KITTI数据集中目标检测算法性能的测评代码。

KITTI中对目标检测算法性能的评估通过绘制precision-recall曲线，并计算平均精度（AP）进行排名。

该文件夹下面包含两个文件夹，其中`devit_object`是KITTI object官网上下载的用于object的测评，`kitti_eval-master`是Github上下载的代码，同样用于KITTI数据集测评，两者的区别在于，`kitti_eval-master`中还包含了`evaluate_object_3d_offline.cpp`可以用于离线测评KITTI数据集检测算法性能，意思是如果你把KITTI object数据集中的training一部分数据划分为test数据，就可以用这个代码自己测评得到结果，否则需要提交到KITTI数据集服务器上测评。



## 测评内容

代码可以进行**2D Object Detection**、**Object Orientation**和**3D Object Detection**测评，取决于给出的检测结果txt文件中各项取值是否有效。

## 测评步骤

以使用`evaluate_object_3d_offline.cpp`为例：

1、将你的目标检测算法检测结果按照KITTI train数据中的label的排列方式保存成txt文件，一张图片一个txt，统一放置在文件夹`detection_results/data`目录下。注意需要对不参与测评的属性项赋无效值，并且需要给出检测置信度值。

**无效值设置（即不参与测评）：**

`alpha = -10`

如果`bbox`的值小于0，则不测评2维检测结果性能；

`location.x = -1000`——不测评BEV

`location.y = -1000`——不测评3D检测性能

2、将数据集真值所有label_02的txt文件放置在路径KITTI/object/label_02下；

3、运行离线测评程序: `./evaluate_object_3d_offline KITTI/object/label_02/ detection_results`;

程序会自动载入`detection_results/data`下所有检测结果，在`detection_results`目录下创建plot目录生成绘制的PR曲线图，完成测评。