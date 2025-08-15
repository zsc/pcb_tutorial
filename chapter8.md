# 第8章：EMC/EMI设计原理

## 本章概述

电磁兼容(EMC)和电磁干扰(EMI)设计是现代高速PCB设计的核心挑战之一。本章将从麦克斯韦方程组出发，建立EMI产生、传播和抑制的完整理论框架。我们将用场论和电路理论的双重视角，深入分析噪声耦合机制，并提供定量化的设计方法。对于AI加速器和高性能计算系统，良好的EMC设计不仅关系到法规认证，更直接影响系统的信号完整性和可靠性。

## 8.1 麦克斯韦方程组在EMI中的应用

### 8.1.1 时变电磁场的基本方程

麦克斯韦方程组的微分形式：

$$\nabla \times \mathbf{E} = -\frac{\partial \mathbf{B}}{\partial t}$$ (法拉第定律)

$$\nabla \times \mathbf{H} = \mathbf{J} + \frac{\partial \mathbf{D}}{\partial t}$$ (安培-麦克斯韦定律)

$$\nabla \cdot \mathbf{D} = \rho$$ (高斯定律)

$$\nabla \cdot \mathbf{B} = 0$$ (磁通连续性)

在PCB环境中，我们主要关注时谐场，可以用相量形式简化：

$$\nabla \times \mathbf{E} = -j\omega\mu\mathbf{H}$$

$$\nabla \times \mathbf{H} = (\sigma + j\omega\epsilon)\mathbf{E}$$

### 8.1.2 PCB中的辐射源

PCB中的辐射主要来自三个源：

1. **差模辐射**：由信号环路产生
   $$E_{DM} = \frac{1.316 \times 10^{-14} \cdot f^2 \cdot A \cdot I}{r}$$
   其中：$f$为频率(Hz)，$A$为环路面积(m²)，$I$为电流(A)，$r$为观测距离(m)

2. **共模辐射**：由电缆天线效应产生
   $$E_{CM} = \frac{1.257 \times 10^{-6} \cdot f \cdot L \cdot I}{r}$$
   其中：$L$为电缆长度(m)

3. **开槽辐射**：由参考平面不连续产生
   $$E_{slot} = \frac{0.023 \cdot V \cdot \sqrt{f}}{r \cdot \sqrt{W}}$$
   其中：$V$为槽两端电压(V)，$W$为槽宽(m)

### 8.1.3 波阻抗与场转换

自由空间波阻抗：
$$Z_0 = \sqrt{\frac{\mu_0}{\epsilon_0}} = 377\Omega$$

近场到远场的转换距离：
$$r = \frac{\lambda}{2\pi} = \frac{c}{2\pi f}$$

在近场区($r < \lambda/2\pi$)：
- 电场源：$Z_{wave} = \frac{377\Omega}{2\pi fr/c}$ (高阻抗)
- 磁场源：$Z_{wave} = \frac{377\Omega \cdot 2\pi fr}{c}$ (低阻抗)

## 8.2 近场与远场辐射模型

### 8.2.1 近场特性分析

在PCB设计中，大部分耦合发生在近场区。近场可进一步分为：

1. **反应近场**：$r < 0.62\sqrt{D^3/\lambda}$
   - 储能占主导，辐射功率可忽略
   - 场强随距离呈$1/r^3$衰减

2. **辐射近场(菲涅尔区)**：$0.62\sqrt{D^3/\lambda} < r < 2D^2/\lambda$
   - 场分布复杂，需要数值计算
   - 开始出现辐射特性

其中$D$为辐射源最大尺寸。

### 8.2.2 PCB走线的辐射模型

**微带线辐射**：
将微带线建模为磁流源，辐射功率：
$$P_{rad} = \frac{60\pi^3}{\lambda^2} \left(\frac{h}{\lambda}\right)^2 I^2 \cdot \left[1 - \left(\frac{\lambda_g}{\lambda}\right)^2\right]$$

其中：
- $h$：介质厚度
- $\lambda_g = \lambda/\sqrt{\epsilon_{eff}}$：导波波长
- $\epsilon_{eff} = \frac{\epsilon_r + 1}{2} + \frac{\epsilon_r - 1}{2} \cdot \frac{1}{\sqrt{1 + 12h/W}}$

**差分对的辐射抑制**：
理想差分对的远场辐射相消，残余辐射：
$$E_{diff} = E_{single} \cdot \frac{\pi s}{\lambda} \cdot \sin\theta$$

其中$s$为线间距，$\theta$为观测角度。

### 8.2.3 通孔阵列的辐射

通孔阵列可视为周期性辐射结构，当满足布拉格条件时产生强辐射：
$$d = \frac{n\lambda}{2\sqrt{\epsilon_r}}$$

其中$d$为通孔间距，$n$为整数。

**通孔天线效应**：
单个通孔的辐射电阻：
$$R_{rad} = 20 \left(\frac{h}{\lambda}\right)^2$$

设计准则：通孔长度应小于$\lambda/20$以避免显著辐射。

### 8.2.4 边缘辐射与镜像原理

PCB边缘的辐射可用镜像法分析：

```
     实际电流 I
         |
    ============= PCB边缘
         |
     镜像电流 -I
```

边缘场强：
$$E_{edge} = \frac{30I}{r} \cdot \sqrt{\frac{h}{\lambda}} \cdot F(\theta)$$

其中$F(\theta)$为方向函数。

## 8.3 共模与差模噪声分析

### 8.3.1 噪声模式的数学描述

对于双导体传输系统，总电流可分解为：

**差模电流**：
$$I_{DM} = \frac{I_1 - I_2}{2}$$

**共模电流**：
$$I_{CM} = \frac{I_1 + I_2}{2}$$

在频域中，噪声功率谱密度：
$$S_{total}(f) = S_{DM}(f) + S_{CM}(f) + 2\sqrt{S_{DM}(f) \cdot S_{CM}(f)} \cdot \cos\phi(f)$$

其中$\phi(f)$为相位相关函数。

### 8.3.2 共模噪声的产生机制

1. **地电位不平衡**：
   $$V_{CM} = I_{ground} \cdot Z_{ground} + L_{ground} \frac{dI_{ground}}{dt}$$

2. **寄生电容耦合**：
   $$I_{CM} = C_{parasitic} \frac{dV_{noise}}{dt}$$

3. **磁场耦合**：
   $$V_{CM} = -\frac{d\Phi}{dt} = -\mu_0 H \cdot A \cdot \omega \cdot \sin(\omega t)$$

### 8.3.3 差模到共模的转换

不对称性导致模式转换，转换系数：
$$S_{cd} = 20\log_{10}\left(\frac{V_{CM,out}}{V_{DM,in}}\right) = 20\log_{10}\left(\frac{\Delta Z}{2Z_0}\right)$$

其中$\Delta Z$为阻抗不平衡。

**PCB中的典型转换源**：
- 走线长度差：$\Delta L$
- 负载不平衡：$\Delta Z_L$
- 介质不均匀：$\Delta \epsilon_r$

转换引起的共模电压：
$$V_{CM} = V_{DM} \cdot \frac{\beta \Delta L}{2}$$

其中$\beta = 2\pi/\lambda$为相位常数。

### 8.3.4 共模扼流圈设计

共模阻抗：
$$Z_{CM} = j\omega L_{CM} = j\omega \mu_r \mu_0 \frac{N^2 A_e}{l_e}$$

差模阻抗（理想情况）：
$$Z_{DM} = j\omega L_{leakage} \approx 0$$

设计参数：
- 匝数：$N = \sqrt{\frac{Z_{CM} \cdot l_e}{2\pi f \mu_r \mu_0 A_e}}$
- 磁芯选择：高$\mu_r$材料（铁氧体）
- 频率特性：$f_{SRF} = \frac{1}{2\pi\sqrt{L_{CM} \cdot C_{parasitic}}}$

## 8.4 屏蔽效能计算

### 8.4.1 屏蔽理论基础

屏蔽效能(SE)定义：
$$SE = 20\log_{10}\left(\frac{E_i}{E_t}\right) = A + R + B$$

其中：
- $A$：吸收损耗
- $R$：反射损耗
- $B$：多次反射修正

### 8.4.2 平面波屏蔽计算

**吸收损耗**：
$$A = 8.686 \cdot t \cdot \sqrt{\pi f \mu_r \sigma} \text{ (dB)}$$

其中：
- $t$：屏蔽层厚度(m)
- $\sigma$：电导率(S/m)
- $\mu_r$：相对磁导率

**反射损耗**：
对于平面波($Z_{wave} = 377\Omega$)：
$$R = 20\log_{10}\left(\frac{Z_{wave}}{4Z_{shield}}\right)$$

屏蔽层阻抗：
$$Z_{shield} = \sqrt{\frac{j\omega\mu}{\sigma}}$$

**趋肤深度**：
$$\delta = \frac{1}{\sqrt{\pi f \mu \sigma}}$$

当$t > 3\delta$时，可忽略多次反射。

### 8.4.3 近场屏蔽计算

**电场源(高阻抗)**：
$$SE_E = 20\log_{10}\left(\frac{1}{2\pi f\epsilon_0 r}\right) + 20\log_{10}\left(\frac{1}{3Z_{shield}}\right) + A$$

**磁场源(低阻抗)**：
$$SE_H = 20\log_{10}\left(\frac{r}{2\mu_r t}\right) + A$$

低频磁屏蔽主要依靠高磁导率材料。

### 8.4.4 孔缝泄漏分析

**单个圆孔**：
$$SE_{aperture} = 20\log_{10}\left(\frac{\lambda}{2d}\right) - 20\log_{10}\left[1 + 2.7\left(\frac{d}{\lambda}\right)^2\right]$$

其中$d$为孔径。

**矩形缝隙**：
最大泄漏发生在$L = \lambda/2$时（谐振）
$$SE_{slot} = 20\log_{10}\left(\frac{\lambda}{2L}\right) - 10\log_{10}(n)$$

其中$L$为缝隙长度，$n$为缝隙数量。

**孔阵屏蔽**：
$$SE_{array} = SE_{single} - 20\log_{10}\left(\sqrt{n}\right) + C$$

修正因子$C$取决于孔间距：
- $s > \lambda/2$：$C = 0$
- $s < \lambda/10$：$C = 10\log_{10}(s/d)$

### 8.4.5 PCB级屏蔽设计

**屏蔽罩设计**：
谐振频率：
$$f_{mn} = \frac{c}{2} \sqrt{\left(\frac{m}{a}\right)^2 + \left(\frac{n}{b}\right)^2}$$

其中$a$、$b$为屏蔽罩尺寸，$m$、$n$为模式数。

**接地过孔墙**：
等效屏蔽效能：
$$SE_{via} = 20\log_{10}\left(\frac{\lambda}{2\pi s}\right)$$

设计准则：过孔间距$s < \lambda/20$。

## 8.5 滤波器设计与接地策略

### 8.5.1 EMI滤波器设计

**滤波器拓扑选择**：

1. **L型滤波器**：
   插入损耗：
   $$IL = 20\log_{10}\left|1 + \frac{Z_s + Z_L}{Z_f}\right|$$
   
   其中$Z_f$为滤波器阻抗。

2. **π型滤波器**：
   传递函数：
   $$H(s) = \frac{1}{1 + s^2LC + s(L/R_L + 2RC)}$$
   
   转折频率：
   $$f_c = \frac{1}{2\pi\sqrt{LC}}$$

3. **T型滤波器**：
   适用于高阻抗源和负载。

**寄生参数影响**：
实际电容的阻抗：
$$Z_C = \frac{1}{j\omega C} + ESR + j\omega ESL$$

自谐振频率：
$$f_{SRF} = \frac{1}{2\pi\sqrt{ESL \cdot C}}$$

设计准则：工作频率应低于$f_{SRF}/3$。

### 8.5.2 差分模式滤波器

**X电容设计**：
所需电容值：
$$C_X = \frac{I_{noise}}{2\pi f V_{limit}}$$

**差模电感**：
$$L_{DM} = \frac{Z_0}{2\pi f_c}$$

其中$Z_0$为特性阻抗，$f_c$为截止频率。

### 8.5.3 共模滤波器设计

**Y电容限制**（安全考虑）：
漏电流限制：
$$I_{leakage} = 2\pi f \cdot C_Y \cdot V_{line} < I_{max}$$

典型值：$C_Y < 4700pF$（医疗设备更严格）。

**共模电感优化**：
最优匝数比：
$$\frac{N_1}{N_2} = \sqrt{\frac{Z_{source}}{Z_{load}}}$$

### 8.5.4 接地系统设计

**单点接地**（低频，$f < 1MHz$）：
接地阻抗：
$$Z_{ground} = R_{DC} + j\omega L_{ground}$$

设计准则：$L_{ground} < \frac{0.05\lambda}{2\pi}$

**多点接地**（高频，$f > 10MHz$）：
网格接地阻抗：
$$Z_{mesh} = \frac{Z_0}{n} \cdot \tanh(\gamma l)$$

其中$n$为并联路径数，$\gamma$为传播常数。

**混合接地**（1MHz < f < 10MHz）：
使用电容隔离：
$$C_{isolation} = \frac{1}{2\pi f \cdot 10\Omega}$$

### 8.5.5 PCB接地设计准则

1. **星形接地**：
   ```
        模拟地 ----*
                   |
        数字地 ----+---- 单点
                   |
        功率地 ----*
   ```

2. **地平面分割**：
   槽宽影响：
   $$Z_{slot} = 60 \ln\left(\frac{4h}{w}\right)$$
   
   其中$h$为板厚，$w$为槽宽。

3. **接地过孔设计**：
   过孔电感：
   $$L_{via} = 0.2h \left[\ln\left(\frac{4h}{d}\right) + 1\right] \text{ (nH)}$$
   
   并联过孔：
   $$L_{parallel} = \frac{L_{via}}{n} + M$$
   
   其中$M$为互感。
