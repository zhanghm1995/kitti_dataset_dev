# 介绍
源码地址：
https://github.com/kuixu/kitti_object_vis

本项目主要是Python和Jupyter写的用来读取KITTI **object**数据并进行可视化的。
可视化内容：

- 3D点云及点云中目标3D包围框
- 图像及图像中目标2D包围框和3D包围框
- 激光雷达俯视图及2D包围框
- 点云在图像上的投影结果

## 配置

系统环境：

```
Ubuntu 18.04
Python 3.6.9
```

在Python虚拟环境中安装：

```
pip install mayavi vtk pillow pyqt5 opencv-python
```



## 运行
主程序为`kitti_object.py`：

**查看帮助：**

```
python kitti_object.py -h
```

结果为：

```shell
usage: kitti_object.py [-h] [-d N] [-i N] [-p] [-s] [--split SPLIT] [-l N]
                       [-e N] [-r N] [--gen_depth] [--vis] [--depth]
                       [--img_fov] [--const_box] [--save_depth] [--pc_label]
                       [--dtype64] [--show_lidar_on_image]
                       [--show_lidar_with_depth] [--show_image_with_boxes]
                       [--show_lidar_topview_with_boxes]

KIITI Object Visualization

optional arguments:
  -h, --help            show this help message and exit
  -d N, --dir N         input (default: data/object)
  -i N, --ind N         input (default: data/object)
  -p, --pred            show predict results
  -s, --stat            stat the w/h/l of point cloud in gt bbox
  --split SPLIT         use training split or testing split (default:
                        training)
  -l N, --lidar N       velodyne dir (default: velodyne)
  -e N, --depthdir N    depth dir (default: depth)
  -r N, --preddir N     predicted boxes (default: pred)
  --gen_depth           generate depth
  --vis                 show images
  --depth               load depth
  --img_fov             front view mapping
  --const_box           constraint box
  --save_depth          save depth into file
  --pc_label            5-verctor lidar, pc with label
  --dtype64             for float64 datatype, default float64
  --show_lidar_on_image
                        project lidar on image
  --show_lidar_with_depth
                        --show_lidar, depth is supported
  --show_image_with_boxes
                        show lidar
  --show_lidar_topview_with_boxes
                        show lidar topview
```

**设置数据路径：**

默认object数据所在路径为data/object，程序已经放置了3帧结果，如果想要自己指定路径，使用-d参数：

```
python kitti_object.py -d /media/zhanghm/zhanghm_ssd/Datasets/KITTI_archive/object
```

默认载入training数据，修改为testing数据：

```
python kitti_object.py --split testing -d /media/zhanghm/zhanghm_ssd/Datasets/KITTI_archive/object
```

**只显示点云：**

```
python kitti_object.py --show_lidar_with_depth --img_fov --const_box --vis
```

其中img_fov参数限定只显示相机范围内点云；

**显示鸟瞰图：**

```
python kitti_object.py --show_lidar_with_depth --img_fov --const_box --vis --show_lidar_topview_with_boxes
```

**显示点云和图片：**

```
python kitti_object.py --show_lidar_with_depth --img_fov --const_box --vis --show_image_with_boxes
```

**显示点云和点云投影到图像：**

```
python kitti_object.py --show_lidar_with_depth --img_fov --const_box --vis --show_lidar_on_image
```

在终端按下键盘任意键读取下一帧，输入`killall`回车结束。