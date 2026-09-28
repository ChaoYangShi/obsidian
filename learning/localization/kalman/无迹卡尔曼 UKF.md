>解决高度非线性系统中的状态估计问题.可以做到近似3阶泰勒展开的精度

在EKF中,面对非线性极强的方程,线性化会截断高阶项,导致严重的误差

### 计算过程
#### 1. 采样 Sigma 点 (缩放对称采样)
对于 $n$ 维的状态向量,通常采样 $2n+1$ 个 $\sigma$ 点
- 第 $0$ 个点 (中心点)
	$X_0 = \mu$
- 第 $1$  到 $n$ 个点 (正向偏移)
	$X_i = \mu + (\sqrt{(n+\lambda)P})_i, i = i, ..., n$
- 第 $n+1$ 到 $2n$ 个点 (反向偏移)
	$X_{i+n} = \mu - (\sqrt{(n+\lambda)P})_i,i = 1,...,n$
比例缩放系数 $\alpha,\beta,\kappa$
$$
\lambda = \alpha^2(n + \kappa) - n
$$
- $\alpha$ 控制主散步范围, $\alpha$ 越大,扩散的越远,通常设置为 $10^{-3} \sim 1$  
- $\kappa$ 辅助缩放函数,保证半正定性,通常为 $0$ 或者  $3-n$ 
- $\beta$ 先验分布信息,用于在计算协方差权重时融入对原分布状态的先验知识,状态变量如果是高斯分布, $\beta = 2$ 是最优的
权重分配
- 中心点权重
$$
\begin{aligned}
W_m^{(0)} &= \frac{\lambda}{n+\lambda} \\
W_c^{(0)} & = \frac{\lambda}{n+\lambda} + (1 - \alpha^2 + \beta)
\end{aligned}
$$
- 其余点权重
$$
W_m^{(i)} = W_c^{(i)} = \frac{1}{2(n+\lambda)}, i = 1,...,2n
$$
#### 2. 预测
将 $2n+1$ 个 $\sigma$ 点带入真实的非线性状态转移方程 $f(x)$ ,得到一组全新的预测点集合 $Y_i$ 
$Y_i = f(x_i)$
对 $Y_i$ 个点进行加权求和,重新预测均值 $\mu^-$ 和先验方差 $P^-$ 
$$
\begin{aligned}
\mu^- &= \Sigma_{i=0}^{2n} W_m^{(i)}Y_i \\
P^- &= \Sigma_{i=0}^{2m}W_c^{(i)}(Y_i - \mu^-)(Y_i-\mu^-)^T+Q
\end{aligned}
$$
其中, $Q$ 为过程噪声,$W_m$ 和 $W_c$ 为对应的均值和协方差权重
#### 3. 观测
将预测出的 $\sigma$ 点,带入真实的非线性观测方程 $h(x)$ ,得到预测观测点 $Z_i$
$Z_i = h(Y_i)$
计算观测均值 $z_{pred}$ ,观测协方差 $P_{zz}$ (需要加上测量噪声 $R$ ),以及状态于观测的互协方差 $P_{xz}$ 
$$
\begin{aligned}
z_{pred} &= \Sigma W_m^{(i)}Z_i \\
P_{zz} &= \Sigma W_c^{(i)}(Z-z_{pred})(Z-z_{pred})^T + R \\
P_{xz} &= \Sigma W_c^{(i)}(Y_i - \mu^-)(Z - z_{pred})^T
\end{aligned}
$$
#### 4. kalman 增益
$$
\begin{aligned}
K &= P_{xz}P_{zz}^{-1} \\
\mu^+ &= \mu^- + K(z_{real} - z_{pred}) \\
P^+ &= P^- -  KP_{zz}K^T
\end{aligned}
$$

