# 介绍
本项目是Python写的用来读取KITTI raw_data和odometry data数据，但其实从源码里看感觉好像也已经写了tracking数据集的读取，待测试。

## 安装

两种安装方式（不用安装也能直接在本地运行）：

### Using pip

You can install pykitti via pip using
```
pip install pykitti
```

### From source
To install the package from source, simply clone or download the repository to your machine
```
git clone https://github.com/utiasSTARS/pykitti.git
```
and run the provided setup tool
```
cd pykitti
python setup.py install
```

## 配置
假设同时下载好了标定文件，并且目录结果保持和KITTI原始一致。

## 运行
运行环境：

Python3

测试raw data文件夹名为：`2011_09_26_drive_0005_sync`，修改`demos/demo_raw.py`文件中的部分代码：

```python
# Change this to the directory where you store KITTI data
basedir = '/home/zhanghm/Dataset/KITTI/raw_data/'

# Specify the dataset to load
date = '2011_09_26'
drive = '0005'
```

就可以直接运行了。

```python
python3 demos/demo_raw.py
```

## 评价

图片和三维点云都通过`matplotlib`显示，不是很直观，特别是点云看起来不太好交互。