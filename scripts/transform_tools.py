#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
1.定义了函数transform_trans_ypr_to_matrix，表示欧拉角转换成齐次变换矩阵
2.定义了函数transform_trans_quat_to_matrix，表示四元数转换成齐次变换矩阵
3.定义了函数transform_matrix_to_trans_ypr，表示齐次变换矩阵转换成欧拉角
4.定义了函数transform_matrix_to_trans_quat，表示齐次变换矩阵转换成四元数
5.定义了函数quat2ypr，表示四元数转换成欧拉角
6.定义了函数ypr2quat，表示欧拉角转换成四元数
7.定义了函数wrap_to_pi 的函数，其目的是将一个弧度值 rad 调整到 [-π, π] 的范围内
'''

# sudo apt-get install python3-numpy
# sudo apt-get install python3-scipy
# sudo apt-get install python3-matplotlib
# sudo apt-get install python3-pandas
# sudo apt-get install python3-sympy
# sudo apt-get install python3-nose
from scipy.spatial.transform import Rotation as R
import numpy as np
import math

'''
目的是将一个包含平移、偏航角、俯仰角和滚转角的向量转换成一个4x4的齐次变换矩阵,
用于将点或物体从一个坐标系变换到另一个坐标系
'''
def transform_trans_ypr_to_matrix(trans_ypr):
    # 将输入的trans_ypr数组重塑为列向量形式
    trans_ypr = trans_ypr.reshape((-1,1))
    # R.from_euler('ZYX', [y,p,r],degrees=true),
    # 'ZYX' means extrinsic 外旋，固定轴, 'zyx' means intrinsic 内旋，旋转轴, we use extrinsic,
    # it basically means World to Object or Object to World
    '''  
    使用scipy.spatial.transform.Rotation模块创建一个旋转对象r，
    这个对象是通过欧拉角（ZYX（外旋、固定轴）顺序，即偏航-俯仰-滚转）来定义的。
    输入的欧拉角是从弧度转换为度的
    '''
    r = R.from_euler('ZYX', [trans_ypr[3,0]*(180.0/math.pi),trans_ypr[4,0]*(180.0/math.pi),trans_ypr[5,0]*(180.0/math.pi)], degrees=True)
    # r_M = r.as_dcm()
    # 将旋转对象r转换为一个3x3的旋转矩阵
    r_M = r.as_matrix()

    # 初始化一个4x4的零矩阵M，用于构建最终的齐次变换矩阵
    M = np.zeros((4,4))
    # 设置齐次变换矩阵的最后一个元素为1
    M[3, 3] = 1
    # 将3x3的旋转矩阵赋值给齐次变换矩阵的左上角
    M[0:3,0:3] = r_M
    # 将输入的平移向量的x、y、z分量赋值给齐次变换矩阵的右半部分（前3行的最后一列）。
    M[0:3,3:4] = np.array([[trans_ypr[0,0]],[trans_ypr[1,0]],[trans_ypr[2,0]]])
    # 返回构建好的4x4齐次变换矩阵
    return M

'''
目的是将包含平移向量和四元数的输入数据转换成一个4x4的齐次变换矩阵
'''
def transform_trans_quat_to_matrix(trans_quat):  # quat: (x, y, z, w)
    # 将输入的 trans_quat 数组重新塑形为列向量形式。
    # 使用 -1 让 NumPy 自动计算行数，以确保数组是单列的
    trans_quat = trans_quat.reshape((-1,1))
    # 从输入数组中提取前三个元素作为平移向量
    trans = trans_quat[0:3, :]
    # 从输入数组中提取后四个元素作为四元数
    quat = trans_quat[3:7, :]

    # 使用 scipy.spatial.transform.Rotation 模块的 from_quat 函数,根据提供的四元数创建一个旋转对象 r
    r = R.from_quat([quat[0, 0], quat[1, 0], quat[2, 0], quat[3, 0]])
    # r_M = r.as_dcm()
    # 将旋转对象 r 转换为一个3x3的旋转矩阵
    r_M = r.as_matrix()

    # 创建一个4x4的零矩阵 M，用于构建最终的齐次变换矩阵
    M = np.zeros((4,4))
    # 设置齐次变换矩阵的最后一个元素为1
    M[3, 3] = 1
    # 将3x3的旋转矩阵赋值给齐次变换矩阵的左上角部分
    M[0:3,0:3] = r_M
    # 将平移向量的x、y、z分量赋值给齐次变换矩阵的右半部分（前3行的最后一列）
    M[0:3,3:4] = np.array([[trans[0,0]],[trans[1,0]],[trans[2,0]]])
    # 返回构建好的4x4齐次变换矩阵
    return M

'''
将一个4x4的齐次变换矩阵转换为包含平移向量（translation）、偏航角（yaw）、俯仰角（pitch）和滚转角（roll）的向量
'''
# 定义了函数 transform_matrix_to_trans_ypr，接受一个参数 M(一个4x4的齐次变换矩阵)
def transform_matrix_to_trans_ypr(M):
    # 从齐次变换矩阵 M 中提取左上角的3x3子矩阵，这个子矩阵表示旋转部分
    r_M = M[0:3,0:3]
    # 从齐次变换矩阵 M 中提取右半部分的前3个元素，这些元素表示平移向量
    trans = M[0:3,3:4]

    # r = R.from_dcm(r_M)
    # 使用 scipy.spatial.transform.Rotation 模块的 from_matrix 函数根据3x3旋转矩阵 r_M 创建一个旋转对象 r。
    r = R.from_matrix(r_M)
    # 将旋转对象 r 转换为欧拉角，使用的是'ZYX'顺序(外旋,固定轴)
    ypr = r.as_euler('ZYX')

    # 创建一个数组 trans_ypr，将平移向量和转换得到的欧拉角组合起来。
    # 这个数组有6行1列，分别表示x、y、z平移和偏航、俯仰、滚转角。
    trans_ypr = np.array([[trans[0,0]],[trans[1,0]],[trans[2,0]],[ypr[0]],[ypr[1]],[ypr[2]]])
    # 返回包含平移和欧拉角的数组
    return trans_ypr

'''
目的是从一个4x4的齐次变换矩阵中提取旋转部分，将其从矩阵形式转换为四元数形式，同时保留平移向量
'''
# 定义了函数 transform_matrix_to_trans_ypr，接受一个参数 M(一个4x4的齐次变换矩阵)
def transform_matrix_to_trans_quat(M):  # quat: (x, y, z, w)
    # 从齐次变换矩阵 M 中提取左上角的3x3子矩阵 r_M，这个子矩阵表示旋转部分。
    r_M = M[0:3,0:3]
    # 从齐次变换矩阵 M 中提取右半部分的前3个元素 trans，这些元素表示平移向量
    trans = M[0:3,3:4]

    # r = R.from_dcm(r_M)
    # 使用 scipy.spatial.transform.Rotation 模块的 from_matrix 函数根据3x3旋转矩阵 r_M 创建一个旋转对象 r
    r = R.from_matrix(r_M)
    # 将旋转对象 r 转换为四元数形式
    quat = r.as_quat()

    # 创建一个数组 trans_quat，将平移向量和转换得到的四元数组合起来。
    # 这个数组有7行1列，分别表示x、y、z平移和四元数的x、y、z、w分量
    trans_quat = np.array([[trans[0,0]],[trans[1,0]],[trans[2,0]],[quat[0]],[quat[1]],[quat[2]],[quat[3]]])
    # 返回包含平移向量和四元数的数组
    return trans_quat


'''
目的是将一个四元数转换为等效的欧拉角表示，具体是偏航角（yaw）、俯仰角（pitch）和滚转角（roll），使用的是'ZYX'的外旋顺序
'''
# 定义了 quat2ypr 函数，接受一个四元数参数 quat，通常表示为一个包含实部（w）和虚部（x, y, z）的向量
def quat2ypr(quat): # quat: (x, y, z, w)
    # 将输入的四元数 quat 转换为列向量形式。
    # 这里使用 -1 让 NumPy 自动计算行数，确保数组是单列的。
    # 为了确保四元数的输入格式与 R.from_quat 函数的要求一致。
    quat = quat.reshape((-1, 1))
    # in R.from_quat(quat), quat: (x, y, z, w)
    # 使用 scipy.spatial.transform.Rotation 模块的 from_quat 函数根据提供的四元数创建一个旋转对象 r。
    r = R.from_quat([quat[0,0], quat[1,0], quat[2,0], quat[3,0]])
    # 将旋转对象 r 转换为欧拉角， 'ZYX' 顺序表示围绕固定轴Z轴、Y轴和X轴旋转。
    ypr = r.as_euler('ZYX') # list [y p r]

    # 得到的欧拉角 ypr 转换为一个3x1的NumPy数组 ypr_np。
    # 这个数组包含了偏航角、俯仰角和滚转角，分别对应于索引0、1和2。
    ypr_np = np.array([[ypr[0]],[ypr[1]],[ypr[2]]])

    # 返回包含偏航角、俯仰角和滚转角的NumPy数组
    return ypr_np

'''
目的是将一组欧拉角（偏航角yaw、俯仰角pitch、滚转角roll）转换为对应的四元数表示
'''
def ypr2quat(ypr):
    # 将输入的欧拉角 ypr 转换为列向量形式
    ypr = ypr.reshape((-1, 1))

    # 使用 scipy.spatial.transform.Rotation 模块的 from_euler 函数根据提供的欧拉角创建一个旋转对象 r。
    # 指定的 'ZYX' 顺序表示旋转是先绕Z轴旋转偏航角，然后绕Y轴旋转俯仰角，最后绕X轴旋转滚转角。
    r = R.from_euler('ZYX', [ypr[0,0], ypr[1,0], ypr[2,0]])
    # in R.as_quat(quat), quat: (x, y, z, w)
    # 将旋转对象 r 转换为四元数形式
    quat = r.as_quat()

    # 将得到的四元数 quat 转换为一个2x4的NumPy数组 quat_np。这个数组包含了四元数的x、y、z分量和实部w。
    quat_np = np.array([[quat[0]],[quat[1]],[quat[2]],[quat[3]]])

    # 返回包含四元数的NumPy数组。
    # 返回的四元数格式为 (x, y, z, w)，其中 (x, y, z) 是四元数的虚部，w是实部。
    return quat_np

'''
定义了一个名为 wrap_to_pi 的函数，其目的是将一个弧度值 rad 调整到 [-π, π] 的范围内
'''
# 定义了 wrap_to_pi 函数，接受一个参数 rad，表示角度的弧度值。
def wrap_to_pi(rad):
    # 将输入的弧度值 rad 与 2π 取模，得到一个在 [0, 2π) 范围内的值。
    rad = rad % (math.pi*2.0)
    # 如果弧度值小于或等于 -π，就将 2π 加到 rad 上，将其转换到 [0, π) 范围内。
    if rad<=(-1.0)*math.pi:
        rad = rad + math.pi*2.0
    # 如果弧度值大于 π，就从 rad 中减去 2π，将其转换到 [-π, 0) 范围内。
    elif rad>math.pi:
        rad = rad - math.pi*2.0

    # 返回调整后的弧度值，这个值现在被确保在 [-π, π] 范围内
    return rad