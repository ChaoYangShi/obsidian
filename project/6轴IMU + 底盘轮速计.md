[kalman 滤波](obsidian://open?vault=learning&file=localization%2Fkalman%2Fkalman%20%E6%BB%A4%E6%B3%A2)
### 分析
- 6轴的IMU只有加速度计和陀螺仪,能提供的观测值为 $a_x, a_y, a_z$ , $\omega_x, \omega_y,\omega_z$ ,同时由于没有磁力计,无法对陀螺仪的零漂进行修正
- 没有RTK的情况下,需要进行局部odom建模,机器开机时,车头的方向为 $x$ 轴,基于右手坐标系,机器左边为 $y$ 轴,速度从底盘读取,沿着车头方向,不需要进行分解
- 进行上下坡时需要考虑重力补偿
### 公式推导
定义状态向量
$$
x_k =[p_x,p_y,\theta_k, v_{body},\omega_z,a_x,b_{\omega},b_a]^T
$$
**此处的 $a_x, b_z$ 已经加上了重力补偿和零漂**
状态转移公式
$$
\begin{aligned}
\begin{bmatrix}
v_{body}
\end{bmatrix}_k &= [v_{body}]_{k-1} + a_x \Delta t \\
\theta_k &= \theta_{k-1} + \omega_z\Delta t \\
\begin{bmatrix}
p_x \\
p_y
\end{bmatrix}_k &=\begin{bmatrix}
p_x \\
p_y
\end{bmatrix}_{k-1} + \begin{bmatrix}
cos\theta_k \\
sin\theta_k
\end{bmatrix}v_{body}\Delta t + \frac{1}{2}a_x\Delta t^2 \\
a_x &= a_x \\
\omega_z &= \omega_z \\
b_\omega &= b_\omega \\
b_a &= b_a
\end{aligned}
$$
状态矩阵 $A$
$$
A = \begin{bmatrix}
1&0&0&cos\theta_k\Delta t&0&\frac{1}{2}\Delta t^2&0&0 \\
0&1&0&sin\theta_k\Delta t&0&\frac{1}{2}\Delta t^2&0&0 \\
0&0&1&0&\Delta t&0&0&0 \\
0&0&0&1&0&\Delta t&0&0 \\
0&0&0&0&1&0&0&0 \\
0&0&0&0&0&1&0&0 \\
0&0&0&0&0&0&1&0 \\
0&0&0&0&0&0&0&1
\end{bmatrix}
$$
协方差矩阵 $P_0$
$$
P_0 = \begin{bmatrix}
1e-8&0&0&0&0&0&0&0 \\
0&1e-8&0&0&0&0&0&0 \\
0&0&1e-7&0&0&0&0&0 \\
0&0&0&1e-7&0&0&0&0 \\
0&0&0&0&1e-8&0&0 \\
0&0&0&0&0&1.0&0&0 \\
0&0&0&0&0&0&10&0 \\
0&0&0&0&0&0&0&10
\end{bmatrix}
$$
底盘速度观测矩阵 $H_{odom}$
$$
H_{odom} = \begin{bmatrix}
0&0&0&1&0&0&0&0
\end{bmatrix}
$$
IMU观测矩阵 $H_{imu}$
$$
H_{imu} = \begin{bmatrix}
0&0&0&0&1&0&1&0 \\
0&0&0&0&0&1&0&1
\end{bmatrix}
$$
过程噪声矩阵 $Q$ 
它的对角线元素代表了每个状态变量在一个 $\Delta t$ 周期内可能发生的“意外突变”的方差
**工程调参法则：状态阶数越高，$Q$ 值越大；纯物理积分的低阶状态，$Q$ 值极小**
$$
Q = \begin{bmatrix}
1e-5&0&0&0&0&0&0&0 \\
0&1e-5&0&0&0&0&0&0 \\
0&0&1e-4&0&0&0&0&0 \\
0&0&0&1e-3&0&0&0&0 \\
0&0&0&0&1e-1&0&0 \\
0&0&0&0&0&1e-1&0&0 \\
0&0&0&0&0&0&1e-8&0 \\
0&0&0&0&0&0&0&1e-8
\end{bmatrix}
$$
底盘速度测量噪声矩阵 $R_{odom}$ 
- 不打滑
$$
R_{odom} = \begin{bmatrix}
1e-3
\end{bmatrix}
$$
- 打滑 **无法处理打滑**
>测试过程中发现predict的速度经常会小于底盘反馈的速度,导致直接进入打滑状态,R增大之后,kalman不再相信底盘反馈,而是使用预测速度,导致定位不更新,但是机器始终在执行控制指令向前移动
$$
R_{odom} = \begin{bmatrix}
1e5
\end{bmatrix}
$$

IMU测量噪声矩阵 $R_{imu}$
$$
R_{imu} = \begin{bmatrix}
1e-2 & 0 \\
0 & 1e-3
\end{bmatrix}
$$
