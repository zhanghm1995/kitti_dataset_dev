# pykitti
目前支持raw_data和odometry data。

## Installation

两种安装方式：

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

## Assumptions
假设同时下载好了标定文件，并且目录结果保持和KITTI原始一致。

## Notation
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

### 显示方案

图片和三维点云都通过`matplotlib`显示，不是很直观，特别是点云看起来不太好交互。