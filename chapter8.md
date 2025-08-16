# 第8章：EMC/EMI设计原理

## 章节概述

电磁兼容性（EMC）和电磁干扰（EMI）是现代高速PCB设计中的核心挑战。本章将从麦克斯韦方程组出发，建立EMI产生机理的数学模型，深入分析噪声耦合路径，并提供系统化的抑制策略。我们将用严格的理论分析取代经验法则，让读者能够定量预测和控制电磁干扰问题。

### 学习目标

完成本章学习后，你将能够：
- 从麦克斯韦方程组推导出PCB中的辐射机理
- 建立近场和远场辐射的数学模型，并进行定量计算
- 区分并分析共模和差模噪声的产生机制和传播路径
- 计算屏蔽结构的屏蔽效能，优化屏蔽设计
- 设计有效的EMI滤波器，制定合理的接地策略
- 运用系统化方法解决实际的EMC问题

## 8.1 麦克斯韦方程组在EMI中的应用

电磁干扰的本质是电磁场的非预期传播。要深入理解EMI，必须从麦克斯韦方程组出发，建立PCB环境下的电磁场模型。

### 8.1.1 基本方程与边界条件

麦克斯韦方程组的微分形式：

$$\nabla \times \mathbf{E} = -\frac{\partial \mathbf{B}}{\partial t}$$ （法拉第定律）

$$\nabla \times \mathbf{H} = \mathbf{J} + \frac{\partial \mathbf{D}}{\partial t}$$ （安培-麦克斯韦定律）

$$\nabla \cdot \mathbf{D} = \rho$$ （高斯定律）

$$\nabla \cdot \mathbf{B} = 0$$ （磁通连续性）

其中本构关系为：
- $\mathbf{D} = \epsilon_0 \epsilon_r \mathbf{E}$
- $\mathbf{B} = \mu_0 \mu_r \mathbf{H}$
- $\mathbf{J} = \sigma \mathbf{E}$（欧姆定律）

在PCB设计中，我们主要关注以下边界条件：

1. **理想导体边界**（铜走线表面）：
   $$\mathbf{n} \times \mathbf{E} = 0$$
   $$\mathbf{n} \cdot \mathbf{B} = 0$$

2. **介质界面**（FR4与空气界面）：
   $$\mathbf{n} \times (\mathbf{E}_1 - \mathbf{E}_2) = 0$$
   $$\mathbf{n} \times (\mathbf{H}_1 - \mathbf{H}_2) = \mathbf{J}_s$$

### 8.1.2 时谐场分析

对于正弦稳态场，采用相量表示：
$$\mathbf{E}(\mathbf{r}, t) = \text{Re}[\mathbf{\tilde{E}}(\mathbf{r})e^{j\omega t}]$$

将时谐场代入麦克斯韦方程，得到亥姆霍兹方程：
$$\nabla^2 \mathbf{\tilde{E}} + k^2 \mathbf{\tilde{E}} = 0$$

其中波数 $k = \omega\sqrt{\mu\epsilon}\sqrt{1 - j\frac{\sigma}{\omega\epsilon}}$

在良导体中（$\sigma >> \omega\epsilon$），电磁场呈指数衰减，趋肤深度为：
$$\delta = \sqrt{\frac{2}{\omega\mu\sigma}} = \frac{1}{\sqrt{\pi f \mu \sigma}}$$

对于铜在1GHz时：$\delta \approx 2.1\mu m$

### 8.1.3 辐射源建模

PCB上的辐射源可以分为三类：

1. **电流环路**（磁偶极子）：
   - 辐射功率：$P_{rad} = \frac{\eta_0}{6\lambda^2}(IA)^2$
   - 其中$A$为环路面积，$I$为环路电流

2. **电压驱动的导线**（电偶极子）：
   - 辐射电阻：$R_{rad} = 80\pi^2(\frac{l}{\lambda})^2$
   - 其中$l$为导线长度

3. **开关电源噪声**（混合源）：
   - 频谱包络：$|V_n(f)| = \frac{V_{pk}t_r}{\pi f}$（$f < \frac{1}{\pi t_r}$）
   - 其中$t_r$为上升时间

### 8.1.4 格林函数方法

对于复杂几何结构，使用并矢格林函数求解：

$$\mathbf{E}(\mathbf{r}) = j\omega\mu \int_V \overline{\mathbf{G}}(\mathbf{r}, \mathbf{r'}) \cdot \mathbf{J}(\mathbf{r'})dV'$$

自由空间中的标量格林函数：
$$G(\mathbf{r}, \mathbf{r'}) = \frac{e^{-jk|\mathbf{r} - \mathbf{r'}|}}{4\pi|\mathbf{r} - \mathbf{r'}|}$$

对于多层PCB结构，需要使用谱域格林函数和索末菲积分技术。

## 8.2 近场与远场辐射模型

理解近场和远场的区别对EMI分析至关重要。近场主要涉及电容性和电感性耦合，而远场则表现为电磁波辐射。

### 8.2.1 近场/远场边界判定

近场与远场的分界通常定义为：

$$r = \frac{\lambda}{2\pi} = \frac{c}{2\pi f}$$

更精确的划分考虑源的尺寸$D$：

1. **反应近场**：$r < 0.62\sqrt{\frac{D^3}{\lambda}}$
2. **辐射近场**（菲涅尔区）：$0.62\sqrt{\frac{D^3}{\lambda}} < r < \frac{2D^2}{\lambda}$
3. **远场**（夫琅禾费区）：$r > \frac{2D^2}{\lambda}$

对于典型的PCB（D=30cm）在1GHz时：
- 反应近场：r < 5.4cm
- 辐射近场：5.4cm < r < 60cm
- 远场：r > 60cm

### 8.2.2 电偶极子与磁偶极子辐射

**电偶极子**（短单极天线模型）：

近场（$kr << 1$）：
$$E_r = \frac{I l \cos\theta}{2\pi\epsilon_0 r^2} \cdot \frac{1}{j\omega}$$
$$E_\theta = \frac{I l \sin\theta}{4\pi\epsilon_0 r^3} \cdot \frac{1}{j\omega}$$

远场（$kr >> 1$）：
$$E_\theta = j\frac{kI l \sin\theta}{4\pi} \cdot \frac{e^{-jkr}}{r} \cdot \eta_0$$

**磁偶极子**（小电流环模型）：

远场辐射：
$$E_\phi = \frac{\omega^2 \mu_0 I A \sin\theta}{4\pi c r} e^{-jkr}$$

其中$A$为环路面积。

### 8.2.3 PCB走线的辐射模型

微带线的辐射可以用传输线理论结合天线理论分析：

1. **共模辐射**（主要辐射源）：
   $$E_{max} = \frac{1.26 \times 10^{-6} f I_{cm} l}{r}$$ (V/m)
   
   其中：
   - $f$：频率(Hz)
   - $I_{cm}$：共模电流(A)
   - $l$：走线长度(m)
   - $r$：观察距离(m)

2. **差模辐射**：
   $$E_{max} = \frac{2.6 \times 10^{-7} f^2 I_{dm} A}{r}$$ (V/m)
   
   其中$A$为电流环路面积(m²)

### 8.2.4 天线效应分析

PCB上的非预期天线结构：

1. **I/O电缆**：
   - 谐振频率：$f_n = \frac{nc}{2l}$（n=1,3,5...）
   - 天线增益：$G \approx 1.64$（半波偶极子）

2. **散热片**：
   - 等效为单极天线
   - 辐射效率：$\eta = \frac{R_{rad}}{R_{rad} + R_{loss}}$

3. **过孔阵列**：
   - 形成谐振腔
   - 谐振频率：$f_{mn} = \frac{c}{2}\sqrt{(\frac{m}{a})^2 + (\frac{n}{b})^2}$

## 8.3 共模与差模噪声分析

噪声模式的识别和转换是EMI控制的关键。共模噪声通常是主要的辐射源，而差模噪声主要影响信号完整性。

### 8.3.1 噪声模式转换机制

模式转换发生在不对称结构处：

**差模到共模转换系数**：
$$S_{cd} = \frac{V_{cm}}{V_{dm}} = \frac{Z_{12} - Z_{11}}{Z_{11} + Z_{12} + 2Z_0}$$

其中$Z_{11}$、$Z_{12}$为阻抗矩阵元素。

主要转换机制：
1. 不平衡负载：$\Delta Z_L = Z_{L1} - Z_{L2}$
2. 走线不对称：$\Delta l = l_1 - l_2$
3. 参考平面不连续
4. 过孔残桩效应

### 8.3.2 共模噪声的产生与传播

共模噪声的主要来源：

1. **地弹噪声**：
   $$V_{ground\_bounce} = L_{gnd} \cdot n \cdot \frac{dI}{dt}$$
   
   其中$n$为同时开关的I/O数量

2. **电源噪声耦合**：
   $$I_{cm} = C_{stray} \cdot \frac{dV_{noise}}{dt}$$

3. **不完整回流路径**：
   - 回流路径阻抗：$Z_{return} = R + j\omega L_{gap}$
   - 产生的共模电压：$V_{cm} = I_{signal} \cdot Z_{return}$

共模传播模型：
$$I_{cm}(z) = I_{cm}(0) \cdot e^{-\gamma_{cm}z}$$

其中传播常数$\gamma_{cm} = \alpha_{cm} + j\beta_{cm}$

### 8.3.3 差模噪声的抑制策略

差模噪声抑制技术：

1. **差分信号设计**：
   - 耦合系数：$k = \frac{M}{\sqrt{L_1 L_2}}$
   - 最优间距：$S = 2H$（H为介质厚度）

2. **扭绞对效应**：
   - 噪声抑制比：$NRR = 20\log(\frac{p}{2\pi r})$ (dB)
   - 其中$p$为扭绞节距

3. **平衡滤波器**：
   - 共模抑制比：$CMRR = 20\log(\frac{A_{dm}}{A_{cm}})$ (dB)

### 8.3.4 混合模式S参数分析

将标准S参数转换为混合模式S参数：

$$\begin{bmatrix} S_{dd} & S_{dc} \\ S_{cd} & S_{cc} \end{bmatrix} = \mathbf{M} \cdot \mathbf{S} \cdot \mathbf{M}^{-1}$$

转换矩阵：
$$\mathbf{M} = \frac{1}{\sqrt{2}}\begin{bmatrix} 1 & -1 & 0 & 0 \\ 1 & 1 & 0 & 0 \\ 0 & 0 & 1 & -1 \\ 0 & 0 & 1 & 1 \end{bmatrix}$$

关键指标：
- 差模传输：$|S_{dd21}|$ > -3dB
- 模式转换：$|S_{cd21}|$ < -30dB
- 共模抑制：$|S_{cc21}|$ < -20dB

## 8.4 屏蔽效能计算

电磁屏蔽是EMI控制的最后防线。有效的屏蔽设计需要综合考虑材料特性、结构完整性和频率响应。

### 8.4.1 平面波屏蔽理论

对于垂直入射的平面波，单层屏蔽的屏蔽效能(SE)可分解为：

$$SE = R + A + B$$ (dB)

其中：
- $R$：反射损耗
- $A$：吸收损耗
- $B$：多次反射修正

**反射损耗**：

电场波（高阻抗）：
$$R_E = 20\log\left(\frac{Z_0}{4Z_s}\right) = 20\log\left(\frac{377}{4Z_s}\right)$$ (dB)

磁场波（低阻抗）：
$$R_H = 20\log\left(\frac{Z_s}{4Z_0}\right) = 20\log\left(\frac{Z_s}{4 \cdot 377}\right)$$ (dB)

屏蔽材料的表面阻抗：
$$Z_s = \sqrt{\frac{j\omega\mu}{\sigma + j\omega\epsilon}} \approx \sqrt{\frac{\omega\mu}{2\sigma}}(1+j)$$

**吸收损耗**：
$$A = 8.686 \cdot \frac{t}{\delta} = 8.686t\sqrt{\pi f \mu \sigma}$$ (dB)

其中$t$为屏蔽层厚度，$\delta$为趋肤深度。

**多次反射修正**（当$A < 10$dB时需考虑）：
$$B = 20\log\left|1 - e^{-2t/\delta}(1-\Gamma^2)\right|$$

### 8.4.2 孔缝泄漏分析

实际屏蔽体上的开孔和缝隙是主要的泄漏路径。

**圆孔泄漏**（孔径$d < \lambda/2$）：

屏蔽效能降低：
$$SE_{aperture} = 20\log\left(\frac{\lambda}{2d}\right) - 10\log(n)$$ (dB)

其中$n$为孔的数量。

**矩形缝隙**（长度$l$，宽度$w$）：

当缝隙作为半波谐振器（$l = \lambda/2$）时泄漏最严重：
$$SE_{slot} = 20\log\left(\frac{\lambda}{2l}\right) + 20\log\left(\frac{1}{\sin(\pi w/l)}\right)$$ (dB)

**孔阵屏蔽设计**：

六角形孔阵的截止频率：
$$f_c = \frac{c}{3.41r}$$

其中$r$为孔半径。屏蔽效能：
$$SE_{array} = 27.3\frac{t}{r} + 20\log\left(\frac{f_c}{f}\right)$$ (dB)（$f < f_c$）

### 8.4.3 多层屏蔽设计

多层屏蔽可以显著提高屏蔽效能，特别是在低频磁场屏蔽中。

**双层屏蔽模型**：

总屏蔽效能：
$$SE_{total} = SE_1 + SE_2 + 20\log\left(1 - e^{-2\gamma d}\right)$$

其中$d$为层间距，$\gamma$为空气中的传播常数。

最优层间距（最大化多层反射）：
$$d_{opt} = \frac{\lambda}{4} = \frac{c}{4f}$$

**磁屏蔽设计**：

低频磁场屏蔽使用高磁导率材料：
$$SE_H = 20\log\left(1 + \frac{\mu_r t}{r}\right)$$ (dB)

其中$r$为屏蔽体半径。

组合屏蔽策略：
1. 内层：高磁导率材料（μ-metal），屏蔽低频磁场
2. 外层：高电导率材料（铜），屏蔽高频电场
3. 中间层：吸波材料，减少谐振

### 8.4.4 屏蔽完整性评估

**转移阻抗法**：

评估电缆屏蔽层或导电衬垫的屏蔽效能：
$$Z_t = \frac{V_{induced}}{I_{shield}} = R_{dc} + j\omega L_t$$

屏蔽效能与转移阻抗的关系：
$$SE = 20\log\left(\frac{Z_0}{Z_t}\right)$$ (dB)

**缝隙阻抗模型**：

导电衬垫的接触阻抗：
$$R_{contact} = \frac{\rho_c}{\sqrt{P/H}}$$

其中$\rho_c$为接触电阻率，$P$为压力，$H$为材料硬度。

**屏蔽室谐振**：

矩形屏蔽腔的谐振频率：
$$f_{mnp} = \frac{c}{2}\sqrt{\left(\frac{m}{a}\right)^2 + \left(\frac{n}{b}\right)^2 + \left(\frac{p}{c}\right)^2}$$

品质因数：
$$Q = \frac{3V}{2\delta S} = \frac{3V\sqrt{\pi f \mu \sigma}}{S}$$

## 8.5 滤波器设计与接地策略

EMI滤波器和正确的接地是控制传导干扰的关键技术。本节将介绍滤波器的设计方法和接地系统的优化策略。

### 8.5.1 EMI滤波器拓扑选择

根据噪声源和负载阻抗选择合适的滤波器拓扑：

**阻抗失配原则**：
- 高阻抗源 → 并联电容（低通）
- 低阻抗源 → 串联电感（高通）

**常用拓扑结构**：

1. **L型滤波器**：
   插入损耗：$$IL = 20\log\left|1 + \frac{Z_L}{Z_S} + \frac{Z_L + Z_{load}}{Z_S}\right|$$ (dB)

2. **π型滤波器**（电容输入）：
   截止频率：$$f_c = \frac{1}{2\pi\sqrt{LC_{eq}}}$$
   其中$C_{eq} = C_1||C_2$

3. **T型滤波器**（电感输入）：
   特性阻抗：$$Z_0 = \sqrt{\frac{L_{eq}}{C}}$$
   其中$L_{eq} = L_1 + L_2 + 2M$

**共模扼流圈设计**：

电感值计算：
$$L_{cm} = \frac{\mu_0 \mu_r N^2 A_e}{l_e}$$

其中：
- $N$：匝数
- $A_e$：有效截面积
- $l_e$：有效磁路长度

饱和电流：
$$I_{sat} = \frac{B_{sat} \cdot l_e}{\mu_0 \mu_r N}$$

### 8.5.2 寄生参数的影响

实际元件的寄生参数会严重影响滤波器性能。

**电容的寄生电感**：

自谐振频率：
$$f_{SRF} = \frac{1}{2\pi\sqrt{LC_{parasitic}}}$$

典型值：
- 0805 MLCC：$L_{parasitic} \approx 1$nH
- 1206 MLCC：$L_{parasitic} \approx 1.5$nH

高频阻抗：
$$Z_C(f) = \left|\frac{1}{j\omega C} + j\omega L_{ESL} + R_{ESR}\right|$$

**电感的寄生电容**：

层间电容：
$$C_{parasitic} = \frac{\epsilon_0 \epsilon_r \pi d l}{t}$$

其中$d$为线径，$l$为总长度，$t$为层间距。

**PCB走线的寄生效应**：

微带线电感：
$$L = 0.2l\left[\ln\left(\frac{2l}{w+h}\right) + 0.5 + \frac{w+h}{3l}\right]$$ (nH)

平行走线电容：
$$C = \frac{\epsilon_0 \epsilon_r l w}{h}$$ (pF)

### 8.5.3 接地系统设计

**接地阻抗分析**：

接地导体的阻抗：
$$Z_{gnd} = R_{dc} + j\omega L_{gnd}$$

其中：
$$R_{dc} = \frac{\rho l}{w \cdot t}$$
$$L_{gnd} = 0.2l\left[\ln\left(\frac{2l}{w+t}\right) + 0.5\right]$$ (nH)

**单点接地**（低频，< 1MHz）：

接地点选择原则：
- 最小化地环路面积
- 避免地电流叠加

地线电感影响：
$$V_{noise} = L_{gnd} \frac{dI}{dt}$$

**多点接地**（高频，> 10MHz）：

接地点间距要求：
$$d < \frac{\lambda}{20} = \frac{c}{20f}$$

接地网格谐振：
$$f_{res} = \frac{c}{2\sqrt{\epsilon_r}}\sqrt{\frac{1}{a^2} + \frac{1}{b^2}}$$

### 8.5.4 混合接地策略

针对宽频带系统，采用混合接地策略：

**频率选择性接地**：

使用电容耦合实现高频接地：
$$C_{couple} > \frac{10}{2\pi f_{min} R_{load}}$$

使用电感隔离低频地环路：
$$L_{choke} > \frac{R_{source}}{2\pi f_{max}}$$

**隔离接地技术**：

光耦隔离的CMR（共模抑制）：
$$CMR = 20\log\left(\frac{V_{cm}}{V_{out}}\right) \approx 20\log\left(\frac{1}{\omega C_{iso} R_{out}}\right)$$ (dB)

变压器隔离的漏感：
$$L_{leakage} = L_{primary}\left(1 - k^2\right)$$

**接地平面分割**：

跨分割的信号完整性影响：
$$Z_{trans} = 60\ln\left(\frac{4h}{w}\right) + j\omega L_{gap}$$

桥接电容选择：
$$C_{bridge} = \frac{1}{(2\pi f_{signal})^2 L_{gap}}$$

## 案例研究：5G毫米波模块的EMI抑制方案

### 背景与挑战

5G毫米波模块工作在24-39GHz频段，面临独特的EMI挑战：
- 极高的工作频率导致波长仅为7.7-12.5mm
- 相控阵天线产生强定向辐射
- 高速数字接口（>10Gbps）产生宽带噪声
- 紧凑封装导致强耦合效应

### 系统级EMI分析

**频谱分配与干扰分析**：

主要干扰源：
1. 本振泄漏：28GHz ± 100MHz
2. 镜像频率：2×LO - RF = 27.6GHz
3. 数字时钟谐波：156.25MHz × n

干扰功率预算：
$$P_{interference} = P_{source} - L_{isolation} + G_{coupling}$$

要求隔离度：
$$L_{isolation} > P_{source} - P_{sensitivity} + M_{margin}$$

典型值：L > 80dB

### 毫米波屏蔽设计

**腔体谐振抑制**：

模块尺寸：10mm × 10mm × 2mm
主模谐振：$f_{110} = \frac{c}{2}\sqrt{\frac{1}{100} + \frac{1}{100}} = 21.2$GHz

谐振抑制方法：
1. 吸波材料加载：Q值从500降至50
2. 腔体分割：2.5mm × 2.5mm子腔
3. 模式搅拌器：破坏谐振条件

**天线隔离设计**：

阵元间隔离度：
$$S_{21} = -20\log\left(\frac{\lambda}{2\pi d}\right) - 10\log(D_1 D_2)$$

实测值：
- 相邻阵元：S21 < -25dB
- 对角阵元：S21 < -45dB

### 高速接口EMI控制

**SerDes接口噪声抑制**：

差分阻抗：100Ω ± 5%
回损要求：RL > 15dB @ 14GHz

眼图裕量分配：
- 垂直眼高：> 100mV
- 水平眼宽：> 0.3UI
- 抖动预算：< 0.28UI

**电源噪声隔离**：

PDN目标阻抗：
$$Z_{target} = \frac{\Delta V_{allowed}}{I_{transient}} = \frac{30mV}{2A} = 15m\Omega$$

去耦网络设计：
- 片上：10nF × 20（< 100MHz）
- 封装：100nF × 10（100MHz-1GHz）
- PCB：10μF × 4（> 1GHz）

### 实测结果与优化

**辐射发射测试**（3m暗室）：

频段 | 限值 | 初始 | 优化后
---|---|---|---
1-6 GHz | 54 dBμV/m | 62 | 48
6-40 GHz | 74 dBμV/m | 85 | 71

**关键优化措施**：
1. 增加屏蔽罩通孔密度：间距从5mm减至2mm
2. 优化接地过孔：增加50%数量
3. 边缘镀铜：减少边缘辐射6dB

## 高级话题：超材料在电磁屏蔽中的应用

### 超材料基本原理

超材料通过亚波长周期结构实现特殊的电磁特性：

**负折射率材料**：
$$n = -\sqrt{\epsilon_r \mu_r}$$

当$\epsilon_r < 0$且$\mu_r < 0$时实现。

**频率选择表面(FSS)**：

带阻型FSS的传输系数：
$$T = \frac{1}{1 + j\frac{Z_{FSS}}{2Z_0}}$$

其中FSS阻抗：
$$Z_{FSS} = j\omega L_{eff}\left(1 - \frac{\omega_0^2}{\omega^2}\right)$$

### 电磁带隙(EBG)结构

**蘑菇型EBG设计**：

带隙中心频率：
$$f_0 = \frac{1}{2\pi\sqrt{LC}}$$

其中：
- $L = \mu_0 h$（通孔电感）
- $C = \frac{w \epsilon_0(\epsilon_r + 1)}{\pi}\cosh^{-1}\left(\frac{w+g}{g}\right)$（贴片电容）

带隙宽度：
$$BW = \frac{1}{\eta}\sqrt{\frac{L}{C}}$$

### 吸波超材料

**完美吸波条件**：

输入阻抗匹配：
$$Z_{in} = Z_0 = 377\Omega$$

实现方法：
$$\epsilon_{eff} = \epsilon' - j\epsilon'' = n/Z$$
$$\mu_{eff} = \mu' - j\mu'' = nZ$$

**宽带吸波设计**：

多层渐变结构：
$$\epsilon_r(z) = \epsilon_{r0}\left(1 + \frac{z}{d}\right)^2$$

吸收率：
$$A = 1 - |S_{11}|^2 - |S_{21}|^2$$

### 可调谐屏蔽

**电控超材料**：

变容二极管调谐：
$$f_{res} = \frac{1}{2\pi\sqrt{L(C_0 + C_{var}(V))}}$$

调谐范围：
$$\frac{\Delta f}{f_0} = \frac{1}{2}\frac{C_{var,max} - C_{var,min}}{C_0 + C_{var,min}}$$

**相变材料应用**：

VO₂的电导率变化：
$$\sigma_{metal}/\sigma_{insulator} > 10^3$$

开关比：
$$ON/OFF = 20\log\left(\frac{\sigma_{metal}}{\sigma_{insulator}}\right) > 60dB$$

## 本章小结

本章从麦克斯韦方程组出发，系统地建立了PCB环境下的EMI分析理论框架。主要知识点包括：

### 核心理论
1. **电磁场基础**：从麦克斯韦方程推导出PCB中的辐射机理，建立了时谐场分析方法
2. **辐射模型**：区分近场和远场特性，建立了电偶极子和磁偶极子的定量模型
3. **噪声模式**：深入分析了共模和差模噪声的产生、传播和转换机制

### 关键公式
- 近远场边界：$r = \lambda/(2\pi)$
- 共模辐射：$E_{max} = 1.26 \times 10^{-6} f I_{cm} l / r$ (V/m)
- 屏蔽效能：$SE = R + A + B$ (dB)
- PDN目标阻抗：$Z_{target} = \Delta V_{allowed} / I_{transient}$

### 设计方法
1. **屏蔽设计**：掌握了单层和多层屏蔽的计算方法，理解孔缝泄漏机理
2. **滤波器设计**：学习了EMI滤波器拓扑选择和寄生参数影响
3. **接地策略**：理解了单点、多点和混合接地的适用场景

### 实践要点
- 共模噪声是主要的辐射源，需要重点控制
- 屏蔽完整性比屏蔽材料本身更重要
- 正确的接地策略可以显著改善EMC性能
- 系统级设计比局部优化更有效

## 练习题

### 基础题

**8.1** 一条长度为10cm的PCB走线，载有频率为100MHz、幅度为10mA的共模电流。计算在3m距离处的电场强度。

<details>
<summary>提示</summary>
使用共模辐射公式，注意单位转换。
</details>

<details>
<summary>答案</summary>

使用公式：$E_{max} = 1.26 \times 10^{-6} f I_{cm} l / r$

代入数值：
- f = 100 × 10⁶ Hz
- I_cm = 10 × 10⁻³ A
- l = 0.1 m
- r = 3 m

$E_{max} = 1.26 \times 10^{-6} \times 100 \times 10^6 \times 10 \times 10^{-3} \times 0.1 / 3$
$E_{max} = 4.2 \times 10^{-3}$ V/m = 4.2 mV/m

转换为dBμV/m：
$E_{dB} = 20\log(4.2 \times 10^{-3} \times 10^6) = 72.5$ dBμV/m
</details>

**8.2** 设计一个工作在2.4GHz的屏蔽罩，要求屏蔽效能达到60dB。如果使用1mm厚的铝板（σ = 3.7×10⁷ S/m），计算其屏蔽效能。

<details>
<summary>提示</summary>
分别计算反射损耗和吸收损耗，铝的相对磁导率μᵣ = 1。
</details>

<details>
<summary>答案</summary>

首先计算趋肤深度：
$\delta = 1/\sqrt{\pi f \mu \sigma} = 1/\sqrt{\pi \times 2.4 \times 10^9 \times 4\pi \times 10^{-7} \times 3.7 \times 10^7}$
$\delta = 1.67 \times 10^{-6}$ m = 1.67 μm

吸收损耗：
$A = 8.686 t/\delta = 8.686 \times 0.001 / (1.67 \times 10^{-6}) = 5201$ dB

表面阻抗：
$Z_s = \sqrt{\omega\mu/(2\sigma)} = \sqrt{2\pi \times 2.4 \times 10^9 \times 4\pi \times 10^{-7}/(2 \times 3.7 \times 10^7)}$
$Z_s = 1.6 \times 10^{-3}$ Ω

反射损耗（远场）：
$R = 20\log(377/(4Z_s)) = 20\log(377/(4 \times 1.6 \times 10^{-3})) = 107$ dB

总屏蔽效能：SE = R + A > 5300 dB（远超要求）
</details>

**8.3** 计算直径2mm的圆形通风孔在10GHz时的屏蔽效能损失。如果需要保持40dB的屏蔽效能，最多可以开多少个这样的孔？

<details>
<summary>提示</summary>
使用圆孔泄漏公式，注意孔径必须小于λ/2。
</details>

<details>
<summary>答案</summary>

波长：$\lambda = c/f = 3 \times 10^8 / 10^{10} = 0.03$ m = 30 mm

验证条件：d = 2mm < λ/2 = 15mm ✓

单孔屏蔽效能损失：
$SE_{aperture} = 20\log(\lambda/(2d)) = 20\log(30/(2 \times 2)) = 17.5$ dB

n个孔的总屏蔽效能损失：
$SE_n = 17.5 - 10\log(n)$ dB

要求SE_n ≥ 40 dB：
$17.5 - 10\log(n) \geq 40$
$10\log(n) \leq -22.5$

这是不可能的，说明单孔就会造成严重泄漏。需要使用波导窗或减小孔径。
</details>

### 挑战题

**8.4** 设计一个四阶EMI滤波器，要求在150kHz-30MHz范围内提供60dB的共模衰减，同时差模插损小于1dB@DC-1MHz。给出完整的电路拓扑和元件参数。

<details>
<summary>提示</summary>
考虑使用共模扼流圈配合Y电容，注意安规对Y电容的限制。
</details>

<details>
<summary>答案</summary>

采用两级共模扼流圈 + Y电容结构：

拓扑：L1(CM) - CY1 - L2(CM) - CY2

设计步骤：
1. 截止频率：$f_c = 150kHz / 3 = 50kHz$（留裕量）
2. 每级衰减：30dB
3. 共模电感：$L_{cm} = 10mH$（每级）
4. Y电容（安规限制）：$C_Y ≤ 4.7nF$

验证截止频率：
$f_c = 1/(2\pi\sqrt{L_{cm} \times C_Y}) = 1/(2\pi\sqrt{10 \times 10^{-3} \times 4.7 \times 10^{-9}}) = 23kHz$

差模电感（漏感）：
$L_{dm} = L_{cm} \times (1-k) = 10mH \times 0.01 = 100μH$

差模截止频率：
$f_{dm} = 1/(2\pi\sqrt{100 \times 10^{-6} \times 2 \times 4.7 \times 10^{-9}}) = 232kHz$

满足要求：差模在1MHz以下插损< 1dB
</details>

**8.5** 分析一个尺寸为200mm×150mm的PCB，工作频率1GHz，估算其作为非预期天线的辐射效率和增益。如果PCB上有一条对角线走线carrying 100MHz方波信号（上升时间1ns），计算最坏情况下的辐射场强。

<details>
<summary>提示</summary>
PCB可视为贴片天线，对角线长度决定谐振频率。方波包含多次谐波。
</details>

<details>
<summary>答案</summary>

对角线长度：
$l = \sqrt{200^2 + 150^2} = 250mm = 0.25m$

在1GHz时：
$\lambda = c/f = 0.3m$
$l/\lambda = 0.25/0.3 = 0.83$ （接近谐振）

作为单极天线的辐射电阻：
$R_{rad} = 80\pi^2(l/\lambda)^2 = 80\pi^2 \times 0.83^2 = 543Ω$

辐射效率（假设损耗电阻10Ω）：
$\eta = R_{rad}/(R_{rad} + R_{loss}) = 543/(543 + 10) = 98\%$

天线增益（贴片天线典型值）：
$G ≈ 6-8 dBi$

方波谐波分析：
基波(100MHz)：相对幅度 = 1
3次谐波(300MHz)：相对幅度 = 1/3
5次谐波(500MHz)：相对幅度 = 1/5
...
带宽：$BW = 0.35/t_r = 0.35/1ns = 350MHz$

最强辐射在300MHz（3次谐波，且更接近PCB谐振）：
假设电流10mA，使用共模辐射公式：
$E = 1.26 \times 10^{-6} \times 300 \times 10^6 \times (10/3) \times 10^{-3} \times 0.25 / 3$
$E = 10.5 mV/m = 80.4 dBμV/m$ @3m

这超过了EN55032 Class B限值（40 dBμV/m @300MHz）！
</details>

**8.6** 设计一个5层PCB的层叠结构，要求同时满足：50Ω±5%阻抗控制、良好的EMI屏蔽、最小化串扰。给出具体的层厚度、材料参数和过孔设计。

<details>
<summary>提示</summary>
考虑信号层与参考层的配置，使用仿真工具验证阻抗。
</details>

<details>
<summary>答案</summary>

层叠结构（1.6mm总厚度）：
1. Top (信号) - 35μm Cu
2. GND (参考) - 35μm Cu，间距0.2mm
3. Signal (信号) - 35μm Cu，间距0.5mm
4. Power (电源) - 35μm Cu，间距0.2mm
5. Bottom (信号) - 35μm Cu

材料：FR-4，εᵣ = 4.3，tanδ = 0.02

阻抗计算（微带线，Top层）：
线宽w = 0.35mm，厚度h = 0.2mm
$Z_0 = \frac{87}{\sqrt{\epsilon_r + 1.41}}\ln\left(\frac{5.98h}{0.8w + t}\right) = 50.2Ω$ ✓

带状线（Layer 3）：
线宽w = 0.2mm，h1 = h2 = 0.35mm
$Z_0 = \frac{60}{\sqrt{\epsilon_r}}\ln\left(\frac{4b}{0.67\pi w(0.8 + t/w)}\right) = 49.8Ω$ ✓

过孔设计：
- 信号过孔：0.2mm孔径，0.4mm焊盘
- 反焊盘：0.6mm（20mil）
- 过孔残桩：< 0.2mm（背钻处理）
- 地过孔间距：< λ/20 = 3mm @5GHz

EMI优化：
- 边缘打过孔墙，间距2mm
- 关键信号包地处理
- 层3信号orthogonal routing避免串扰
</details>

**8.7** 推导并分析差分线到单端线的模式转换。如果差分对的长度差为5mm，在1GHz时计算模式转换系数和共模噪声电平。

<details>
<summary>提示</summary>
长度差导致相位差，进而产生共模分量。
</details>

<details>
<summary>答案</summary>

长度差导致的相位差：
$\Delta\phi = \beta \Delta l = \frac{2\pi}{\lambda} \Delta l$

在PCB中（εᵣ = 4.3）：
$v_p = c/\sqrt{\epsilon_r} = 3 \times 10^8 / \sqrt{4.3} = 1.45 \times 10^8 m/s$
$\lambda = v_p/f = 1.45 \times 10^8 / 10^9 = 0.145m$

相位差：
$\Delta\phi = \frac{2\pi \times 0.005}{0.145} = 0.217 rad = 12.4°$

模式转换系数：
$S_{cd} = \sin(\Delta\phi/2) = \sin(6.2°) = 0.108$
$S_{cd}(dB) = 20\log(0.108) = -19.3dB$

如果差模信号为1V：
共模噪声 = 1V × 0.108 = 108mV

这将产生显著的共模辐射：
$E = 1.26 \times 10^{-6} \times 10^9 \times (108 \times 10^{-3} \times I_{trace}) \times l / r$

需要控制长度匹配在λ/20以内（7.25mm @1GHz）。
</details>

**8.8** 设计一个超材料吸波结构，要求在X波段（8-12GHz）实现>20dB的吸收。给出单元结构参数和阵列配置。

<details>
<summary>提示</summary>
使用电阻膜配合周期结构，考虑四分之一波长谐振。
</details>

<details>
<summary>答案</summary>

采用Salisbury屏设计的改进版：

基本结构：
- 顶层：方形贴片阵列，边长a = 7mm
- 电阻膜：377Ω/□（匹配自由空间阻抗）
- 间隔层：泡沫或蜂窝，厚度h
- 底层：金属反射板

设计计算：

中心频率：f₀ = 10GHz
波长：λ₀ = 30mm
间隔层厚度：h = λ₀/4 = 7.5mm

贴片尺寸优化（使用等效电路）：
谐振频率：$f_r = \frac{c}{2a\sqrt{\epsilon_{eff}}}$

其中：$\epsilon_{eff} = \frac{\epsilon_r + 1}{2}$（近似）

带宽增强：
使用多层结构，厚度渐变：
- Layer 1: h₁ = 6mm，R₁ = 200Ω/□
- Layer 2: h₂ = 7.5mm，R₂ = 377Ω/□
- Layer 3: h₃ = 9mm，R₃ = 550Ω/□

仿真结果：
- 8GHz: -22dB
- 10GHz: -35dB
- 12GHz: -25dB

总厚度：22.5mm（0.75λ₀）
重量：~2kg/m²
</details>

## 常见陷阱与错误

### 设计阶段
1. **忽视共模路径**：只关注差模信号，忽略共模噪声是主要辐射源
2. **屏蔽不完整**：有良好的屏蔽材料但忽视缝隙和孔洞
3. **接地环路**：在不同接地点之间形成大面积环路
4. **滤波器位置错误**：将滤波器放在噪声源远端而非近端

### 布局布线
1. **跨越分割**：高速信号跨越地平面分割造成阻抗不连续
2. **电缆布置不当**：I/O电缆靠近高噪声源或形成天线
3. **去耦电容放置**：去耦电容离电源引脚太远，引线电感过大
4. **参考平面切换**：信号在不同层间切换时没有提供回流路径

### 测试验证
1. **测试设置错误**：接地不良或电缆位置影响测试结果
2. **频率遗漏**：只测试基波，忽略谐波频率
3. **近场忽视**：只进行远场测试，忽略近场耦合问题
4. **裕量不足**：刚好满足标准限值，没有考虑生产偏差

### 故障排查
1. **单点思维**：只修改一个问题点，忽视系统性问题
2. **过度屏蔽**：增加屏蔽但不解决根本噪声源
3. **经验主义**：依赖经验法则而不是定量分析
4. **忽视机械因素**：屏蔽罩接触不良、螺丝松动等

## 最佳实践检查清单

### EMI设计审查要点

#### 原理图阶段
- [ ] 识别所有高速信号和时钟
- [ ] 标注噪声源和敏感电路
- [ ] 规划滤波器位置和类型
- [ ] 确定接地策略（单点/多点/混合）
- [ ] 评估电缆和连接器的EMI风险

#### PCB布局阶段
- [ ] 功能模块分区（数字/模拟/电源/RF）
- [ ] 关键信号远离板边（>20倍走线高度）
- [ ] I/O连接器集中放置并良好接地
- [ ] 晶振放置远离I/O和板边
- [ ] 去耦电容紧邻IC电源引脚

#### 布线阶段
- [ ] 高速差分对长度匹配（<5%偏差）
- [ ] 关键信号不跨越平面分割
- [ ] 时钟走线包地或内层走线
- [ ] 回流路径连续性检查
- [ ] 过孔数量和位置优化

#### 屏蔽设计
- [ ] 屏蔽完整性（360°连接）
- [ ] 通风孔小于λ/20
- [ ] 电缆屏蔽层正确端接
- [ ] 导电衬垫压缩比>20%
- [ ] 屏蔽罩与PCB地良好连接

#### 滤波器设计
- [ ] 输入输出物理隔离
- [ ] 共模与差模分别考虑
- [ ] 考虑寄生参数影响
- [ ] 安规要求满足（Y电容<4.7nF）
- [ ] 瞬态响应验证

#### 接地系统
- [ ] 参考平面完整性
- [ ] 模拟地与数字地连接策略
- [ ] 机壳地连接点选择
- [ ] 静电放电路径规划
- [ ] 接地阻抗测量（<5mΩ）

#### 测试验证
- [ ] 预兼容测试计划
- [ ] 辐射发射（RE）测试配置
- [ ] 传导发射（CE）测试设置
- [ ] 抗扰度测试要求
- [ ] 裕量要求（>6dB）