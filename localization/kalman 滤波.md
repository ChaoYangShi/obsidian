### 1 作用
在存在不确定性的情况下,通过融合 **物理模型预测** 和 **传感器实际观测** ,来推算系统当前的最优真实状态
- **物理模型** : 根据控制指令和车辆运动学模型,可以推测出小车的理论位置,但是由于轮胎打滑或者执行器延迟,实际运动位姿和理论预测位姿存在误差,这个叫做 **过程噪声 $Q$**
- **传感器测量** : 通过 GPS, IMU, Lidar获得定位数据,但是任何传感器都有固定的测量误差,漂移,跳变,这个叫做 **测量噪声 $R$**
### 2 核心机制
#### 2.1 预测
$$
\hat{x}_k^- = A \hat{x}_{k-1} + Bu_k
$$
$$
P_k^- = AP_{k-1}A^T + Q
$$
$x_k$ 是状态向量 , $A$ 是状态转移矩阵, $B$ 是控制输入矩阵, $P$ 是协方差矩阵, $Q$ 是过程噪声的协方差矩阵
#### 2.2 更新
$$
K_k = P_k^-H^T(HP_k^-H^T + R)^{-1}
$$
$$
\hat{x}_k = \hat{x}_k^- + K_k(z_k - H \hat{x}_k^-)
$$
$$
P_k = (I - K_kH)P_k^-
$$
$H$ 是观测矩阵, $z_k$ 是传感器观测值, $R$ 是测量噪声的协方差矩阵
### 3 示例
把加速度 $a$ 作为外部输入 $u_k$ ,从而状态变量为
$$
x_k = [p_{x,k}, p_{y,k}, v_{x,k}, v_{y,k}]^T
$$
根据加速度公式
$$
\begin{aligned}
p_{x,k} &= p_{x, k-1} + v_{x,k-1} \times \Delta t + \frac{1}{2}a\Delta t^2 \\
v_{x,k} &= v_{x,k-1} + a \Delta t \\
\end{aligned}
$$
$y$ 方向同理,所以得到状态转移矩阵 $A$
$$
A = \begin{bmatrix}
1&0&\Delta t&0 \\
0&1&0&\Delta t \\
0&0&1&0 \\
0&0&0&1
\end{bmatrix}
$$

得到控制输入矩阵 $B$
$$
B = \begin{bmatrix}
\frac{1}{2}\Delta t^2 \\
\frac{1}{2}\Delta t^2 \\
\Delta t \\
\Delta t
\end{bmatrix}
$$
最后状态转移方程为
$$
\begin{bmatrix}
p_{x,k} \\
p_{y,k} \\
v_{x,k} \\
v_{y,k}
\end{bmatrix} = \begin{bmatrix}
1&0&\Delta t&0 \\
0&1&0&\Delta t \\
0&0&1&0 \\
0&0&0&1
\end{bmatrix}
\begin{bmatrix}
p_{x,k-1} \\
p_{y,k-1} \\
v_{x,k-1} \\
v_{y,k-1}
\end{bmatrix} + \begin{bmatrix}
\frac{1}{2}\Delta t^2 \\
\frac{1}{2}\Delta t^2 \\
\Delta t \\
\Delta t
\end{bmatrix} u_k
$$
