Kabsch / Procrustes 二维刚体配准伪代码

目标：
    已知两组一一对应的点：
        real_points   ：现实雷达坐标系下的点
        mujoco_points ：MuJoCo 坐标系下的对应点

    求一个二维刚体变换，使得：

        p_mujoco ≈ R * p_real + t

    其中：
        R 是二维旋转矩阵
        t 是二维平移向量


算法输入：
    real_points[1...N]
    mujoco_points[1...N]

    其中：
        real_points[i] 和 mujoco_points[i] 是一组对应点
        每个点包含 x, y 两个坐标
        N >= 2，工程中建议 N >= 4


算法输出：
    R
    t
    yaw_offset
    errors


算法流程：

1. 检查输入数据

    如果 real_points 的数量不等于 mujoco_points 的数量：
        报错：两组点数量不一致

    如果点的数量 N 小于 2：
        报错：点数量不足

    如果所有点几乎共线或重合：
        提示：配准结果可能不稳定


2. 计算现实点集的质心

    real_center = (0, 0)

    对 i 从 1 到 N：
        real_center = real_center + real_points[i]

    real_center = real_center / N


3. 计算 MuJoCo 点集的质心

    mujoco_center = (0, 0)

    对 i 从 1 到 N：
        mujoco_center = mujoco_center + mujoco_points[i]

    mujoco_center = mujoco_center / N


4. 对两组点去中心化

    对 i 从 1 到 N：
        real_centered[i] = real_points[i] - real_center
        mujoco_centered[i] = mujoco_points[i] - mujoco_center

    说明：
        这一步是把两组点都移动到原点附近，
        暂时去掉平移影响，只保留旋转关系。


5. 构造协方差矩阵 H

    初始化 2×2 矩阵 H 为零矩阵：

        H = [ 0  0 ]
            [ 0  0 ]

    对 i 从 1 到 N：

        设：
            p = real_centered[i]
            q = mujoco_centered[i]

        累加：

            H = H + p * q^T

    其中：
        p 是 2×1 向量
        q 是 2×1 向量
        q^T 是 q 的转置
        p * q^T 得到一个 2×2 矩阵


6. 对协方差矩阵 H 做 SVD 分解

    对 H 进行奇异值分解：

        H = U * S * V^T

    得到：
        U
        S
        V^T


7. 根据 SVD 结果计算旋转矩阵 R

    R = V * U^T

    其中：
        V 是 V^T 的转置
        U^T 是 U 的转置


8. 检查是否出现镜像反射

    如果 det(R) < 0：

        说明 R 中包含镜像反射，不是合法的二维旋转矩阵。

        修正方法：

            将 V 的最后一列取反

            然后重新计算：

                R = V * U^T

    否则：

        保持 R 不变


9. 计算平移向量 t

    t = mujoco_center - R * real_center

    至此，得到完整的坐标变换：

        p_mujoco = R * p_real + t


10. 从旋转矩阵 R 中提取 yaw 偏移角

    二维旋转矩阵 R 的形式为：

        R = [ cos(theta)  -sin(theta) ]
            [ sin(theta)   cos(theta) ]

    因此：

        yaw_offset = atan2(R[2,1], R[1,1])

    注意：
        这里 R[2,1] 表示第二行第一列
        R[1,1] 表示第一行第一列


11. 计算每个点的配准误差

    对 i 从 1 到 N：

        predicted_mujoco_point[i] = R * real_points[i] + t

        error_vector[i] = mujoco_points[i] - predicted_mujoco_point[i]

        errors[i] = norm(error_vector[i])

    其中：
        norm 表示二维向量长度


12. 计算整体误差指标

    mean_error = 所有 errors 的平均值

    max_error = 所有 errors 中的最大值

    如果 mean_error 和 max_error 都较小：
        说明配准效果较好，可以使用该变换

    如果某个点的 error 明显很大：
        说明该点可能标错，或者对应关系不准确

    如果误差随着距离增大而增大：
        可能存在尺度误差，可以考虑 Sim(2) 方法，即加入缩放因子


13. 使用变换转换新的现实坐标点

    输入现实雷达坐标系中的新点：

        p_real = (x_real, y_real)

    转换到 MuJoCo 坐标系：

        p_mujoco = R * p_real + t


14. 使用变换转换机器人的姿态角

    输入现实雷达坐标系下的 yaw：

        yaw_real

    转换到 MuJoCo 坐标系：

        yaw_mujoco = yaw_real + yaw_offset

    然后将 yaw_mujoco 归一化到 [-pi, pi] 范围内。


最终结果：

    位置转换：

        p_mujoco = R * p_real + t

    姿态转换：

        yaw_mujoco = yaw_real + yaw_offset


简化版伪代码：

    输入 real_points, mujoco_points

    检查两组点数量是否一致
    检查点数量是否足够

    real_center = real_points 的平均值
    mujoco_center = mujoco_points 的平均值

    对每个点 i：
        real_centered[i] = real_points[i] - real_center
        mujoco_centered[i] = mujoco_points[i] - mujoco_center

    H = 零矩阵

    对每个点 i：
        H = H + real_centered[i] * mujoco_centered[i]^T

    对 H 做 SVD 分解：
        H = U * S * V^T

    R = V * U^T

    如果 det(R) < 0：
        将 V 的最后一列取反
        R = V * U^T

    t = mujoco_center - R * real_center

    yaw_offset = atan2(R[2,1], R[1,1])

    对每个点 i：
        predicted[i] = R * real_points[i] + t
        errors[i] = norm(mujoco_points[i] - predicted[i])

    mean_error = errors 的平均值
    max_error = errors 的最大值

    输出：
        R
        t
        yaw_offset
        errors
        mean_error
        max_error