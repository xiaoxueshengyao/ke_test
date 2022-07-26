卡尔曼滤波的简单例子
kf_wendu.py  一维的数据进行卡尔曼滤波，简单的温度检测

kf_x_v.py   二维数据（x,v）进行卡尔曼滤波，或者可以叫数据融合，转载自下链接
https://github.com/liuchangji/2D-Kalman-Filter-Example_Dr_CAN_in_python

Dr Can卡尔曼滤波例子的python实现
例子的简单实现
其实通过调整过程噪声和测量噪声可以发现，当测量噪声方差较小时，后验数据更接近测量值，
当把测量噪声方差调大时，因为有先验的缘故，后验数据也并没有像测量数据那样偏的很大

视频地址：https://www.bilibili.com/video/BV1dV411B7ME/?spm_id_from=333.788.recommend_more_video.1




