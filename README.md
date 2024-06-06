# STM32

####  介绍

本项目开源STM32相关例程和资料 注意：代码仅适用于STM32F1系列，目前在STM32F103C8T6上已完成测试。                                                                
b站教学视频：https://www.bilibili.com/video/BV1A1421671G/?spm_id_from=333.999.0.0&vd_source=c053026cfad6feccbe86fd46e528c91a

**文件说明**

 **car_example** --小车完整模版代码 包括pid角度+速度闭环、mpu6050姿态解算等 

 **template** --供你自己用于开发的STM32空白工程 

 **源码** --包含pid、卡尔曼滤波、灰度循迹的源码文件 可自行添加到工程中

**参考说明：**

1. 特别感谢e芯工作室的gwj学长，逐飞科技以及正点原子，我从中借鉴了许多写代码的思路！
2. 本套驱动库oled和delay部分参考江协科技，特此感谢。