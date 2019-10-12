#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import scipy.misc

np.set_printoptions(threshold='nan')
global saved
saved = 0

def scale_to_255(a, min, max, dtype=np.uint8):
    return (((a - min) / float(max - min)) * 255).astype(dtype)

# 本部分代码之所以引入投影区域大小，引入分辨率res，引入像素值是为了将点云投影转化成图片。如果不要将点云数据转化成图片需要保留点云真实数据是没有必要这样做的。
# 这里的side_range和fwd_range用来限制带投影点云的区域大小，这样利于生成图片。其范围决定了输出图片的尺寸，其值只要比点云数据坐标值的最大值大即可。
# y轴是沿着竖直方向的;x，z轴是在水平面内的。保留x,z；去除y。一般z轴坐标值为正，x和y轴坐标值都有正有负。
def point_cloud_2_birdseye(points,
                           res=0.01,
                           side_range=(-6, 6), # 用于限制x轴坐标值
                           fwd_range = (-0.1, 6), # 用于限制z轴坐标值
                           height_range=(-0.5, 1.1),
                           ):
    x_points = points[:, 0] #x
    y_points = points[:, 1] #y
    z_points = points[:, 2] #z
    # 只取某个区域内的点云数据
    # 注意这里逻辑有点绕，会对限制坐标取相反数
    f_filt = np.logical_and((z_points > fwd_range[0]), (z_points < fwd_range[1]))
    s_filt = np.logical_and((x_points > -side_range[1]), (x_points < -side_range[0]))
    filter = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filter).flatten()
    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]

    # 选择有效高度，去除地板上的点云
    height_filt = np.logical_and((y_points > height_range[0]), (y_points < height_range[1]))
    indices = np.argwhere(height_filt).flatten()
    x_points = x_points[indices]
    y_points = y_points[indices]
    z_points = z_points[indices]

    # 计算投影之后，点云对应二维像素的坐标
    z_img = (-z_points / res).astype(np.int32)
    x_img = (-x_points / res).astype(np.int32)

    x_img -= int(np.floor(side_range[0] / res))
    z_img += int(np.ceil(fwd_range[1] / res))
    # print x_img
    # 将高度转换为灰度值，来用二维图像体现三维高度信息(暂时注释了，转而采用投影点都设为最大像素的方法)
    # pixel_values = np.clip(a=y_points,
    #                        a_min=height_range[0],
    #                        a_max=height_range[1])

    # pixel_values = scale_to_255(pixel_values,
    #                                 min=height_range[0],
    #                                 max=height_range[1])
    # print pixel_values
    pixel_values = 255
    # 确定图像尺寸，初始化图像矩阵
    x_max = 1 + int((side_range[1] - side_range[0]) / res)
    z_max = 1 + int((fwd_range[1] - fwd_range[0]) / res)
    im = np.zeros([z_max, x_max], dtype=np.uint8)

    im[z_img, x_img] = pixel_values
    # 显示原点
    x_or = -int(np.floor(side_range[1] / res))
    y_or = int(np.ceil(fwd_range[1] / res))
    im[y_or, x_or] = pixel_values
    return im

def callback(lidar):
    global saved
    if saved == 0:
        print 'begain'
        lidar = pc2.read_points(lidar)
        points = np.array(list(lidar))
        im = point_cloud_2_birdseye(points)
        scipy.misc.imsave('/home/sxy/r.png', im)
        saved = 1
        reason = 'end'
        rospy.signal_shutdown(reason)

def cloud_subscribe():
    rospy.init_node('cloud_subscribe_node')
    rospy.Subscriber("/xtion/depth_registered/points", PointCloud2, callback)
    rospy.spin()

if __name__ == '__main__':
    cloud_subscribe()