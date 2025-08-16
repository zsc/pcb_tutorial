# 第7章：电源完整性与去耦设计

## 本章概述

电源完整性(Power Integrity, PI)是现代高速PCB设计的核心挑战之一。随着处理器频率提升、电流密度增加、电压裕量缩小，电源分配网络(PDN)的设计已从简单的"供电"功能演变为复杂的系统工程问题。本章将从理论基础出发，建立PDN的数学模型，深入分析去耦策略、噪声抑制机制，并通过算法优化实现高性能电源设计。

对于AI加速器、GPU等高功耗芯片，瞬态电流可达数百安培，电压波动必须控制在±5%以内。这要求我们不仅理解PDN的频域特性，还需要掌握时域响应、空间分布、热电耦合等多物理场效应。本章将用严格的数学推导和定量分析，帮助读者建立对电源完整性的深刻理解。

### 学习目标

完成本章学习后，你将能够：

1. 建立PDN的精确数学模型，计算不同频率下的阻抗特性
2. 推导目标阻抗公式，设计满足纹波要求的去耦网络
3. 运用优化算法确定去耦电容的值、数量和位置
4. 分析SSN的产生机理，定量评估其影响
5. 识别电源平面谐振模式，设计有效的抑制策略
6. 将理论知识应用于实际的高性能计算系统设计

## 7.1 电源分配网络(PDN)建模

### 7.1.1 PDN的层次结构

现代PDN是一个多尺度、多层次的复杂系统，从芯片内部到系统级电源，跨越了8个数量级的频率范围（DC到10GHz）。我们可以将PDN分解为以下层次：

```
VRM (电压调节模块) → PCB电源平面 → 封装基板 → 片上电源网格 → 晶体管
    ↓                    ↓              ↓            ↓           ↓
  DC-1MHz            1-100MHz      10MHz-1GHz    100MHz-10GHz  >1GHz
```

每个层次都有其特征阻抗和响应频率，理解这种层次化结构是设计高效PDN的基础。

### 7.1.2 集总参数模型

在低频段（<100MHz），PDN可以用集总RLCG网络建模。考虑一个简化的PDN段：

```
     L_trace           R_trace
  ───────────────────────────────
         │                │
         C_bulk          C_decap
         │                │
  ───────────────────────────────
            GND平面
```

其阻抗可表示为：

$$Z_{PDN}(s) = R + sL + \frac{1}{sC_{total}} = R + sL + \frac{1}{s(C_{bulk} + C_{decap})}$$

其中：
- $R$ = 走线和过孔的串联电阻
- $L$ = 走线和过孔的串联电感
- $C_{total}$ = 所有并联电容的总和

### 7.1.3 分布参数模型

当频率升高，波长与PCB尺寸可比时（λ/10 < PCB尺寸），必须使用传输线理论。电源/地平面对可视为平行板传输线：

特性阻抗：
$$Z_0 = \sqrt{\frac{L_{plane}}{C_{plane}}} = \frac{h}{\sqrt{\epsilon_r}} \cdot \sqrt{\frac{\mu_0}{\epsilon_0}} = \frac{377h}{\sqrt{\epsilon_r} \cdot w}$$

其中：
- $h$ = 平面间距（介质厚度）
- $w$ = 平面宽度
- $\epsilon_r$ = 介质相对介电常数

传播常数：
$$\gamma = \alpha + j\beta = \sqrt{(R + j\omega L)(G + j\omega C)}$$

对于低损耗情况：
$$\beta \approx \omega\sqrt{LC} = \frac{\omega}{v_p}$$

其中传播速度：
$$v_p = \frac{1}{\sqrt{LC}} = \frac{c}{\sqrt{\epsilon_r \mu_r}}$$

### 7.1.4 平面谐振模型

电源/地平面对在特定频率会产生谐振，形成驻波。对于矩形平面（长度a，宽度b），谐振频率为：

$$f_{mn} = \frac{c}{2\sqrt{\epsilon_r}}\sqrt{\left(\frac{m}{a}\right)^2 + \left(\frac{n}{b}\right)^2}$$

其中m, n为模式编号（0, 1, 2...）。

第一个谐振频率（基模）：
$$f_{10} = \frac{c}{2a\sqrt{\epsilon_r}}$$

例如，对于100mm×100mm的平面，εr=4.4：
$$f_{10} = \frac{3×10^8}{2×0.1×\sqrt{4.4}} ≈ 714MHz$$

### 7.1.5 多端口网络模型

实际PDN有多个电源引脚和负载点，需要用多端口S参数或Z参数描述：

$$[V] = [Z][I]$$

其中Z矩阵的元素：
$$Z_{ij} = \frac{V_i}{I_j}\bigg|_{I_k=0, k≠j}$$

自阻抗$Z_{ii}$表示端口i的输入阻抗，互阻抗$Z_{ij}$表示端口j到端口i的转移阻抗。

### 7.1.6 时域响应分析

PDN的时域响应对理解瞬态噪声至关重要。给定阶跃电流激励$I(t) = I_0 \cdot u(t)$，电压响应为：

$$V(t) = \mathcal{L}^{-1}\{Z(s) \cdot I(s)\} = I_0 \cdot \mathcal{L}^{-1}\left\{\frac{Z(s)}{s}\right\}$$

对于RLC二阶系统：
$$Z(s) = R + sL + \frac{1}{sC}$$

阶跃响应（欠阻尼情况）：
$$V(t) = I_0 \cdot Z_0 \cdot e^{-\alpha t}\left[\cos(\omega_d t) + \frac{\alpha}{\omega_d}\sin(\omega_d t)\right]$$

其中：
- 阻尼系数：$\alpha = \frac{R}{2L}$
- 阻尼振荡频率：$\omega_d = \sqrt{\omega_0^2 - \alpha^2}$
- 自然频率：$\omega_0 = \frac{1}{\sqrt{LC}}$

## 7.2 目标阻抗计算与频域分析

### 7.2.1 目标阻抗的物理意义

目标阻抗(Target Impedance, $Z_{target}$)定义了PDN在特定频率范围内必须满足的最大阻抗值，以确保电源噪声不超过允许范围。其基本公式：

$$Z_{target} = \frac{\Delta V_{allowed}}{I_{transient}}$$

其中：
- $\Delta V_{allowed}$ = 允许的电压波动（通常为$V_{DD} \times ripple\%$）
- $I_{transient}$ = 最大瞬态电流变化

### 7.2.2 频率相关的目标阻抗

实际应用中，瞬态电流具有频谱特性，需要考虑频率相关的目标阻抗：

$$Z_{target}(f) = \frac{V_{DD} \times ripple\%}{I_{transient}(f)}$$

瞬态电流频谱可通过傅里叶变换获得：
$$I_{transient}(f) = \mathcal{F}\{i(t)\}$$

对于周期性开关电流（如时钟同步电路）：
$$i(t) = I_{DC} + \sum_{n=1}^{\infty} I_n \cos(2\pi nf_{clk}t + \phi_n)$$

其频谱在时钟频率的谐波处有峰值。

### 7.2.3 多负载系统的目标阻抗

当系统有多个负载时，需要考虑最坏情况下的同步开关：

$$Z_{target} = \frac{V_{DD} \times ripple\%}{\sum_{i=1}^{N} I_{i,max} \cdot SF_i}$$

其中：
- $N$ = 负载数量
- $I_{i,max}$ = 第i个负载的最大电流
- $SF_i$ = 同步因子（0到1之间）

### 7.2.4 频域扫描与优化

PDN阻抗的频域特性通过网络分析仪测量或仿真获得。典型的阻抗曲线呈现多个谐振峰：

```
|Z(f)|
  ↑
  │     ╱╲
  │    ╱  ╲    ╱╲
  │   ╱    ╲  ╱  ╲
  │  ╱      ╲╱    ╲___Z_target
  │ ╱                
  └─────────────────→ f
   DC   f1   f2   f3
```

优化目标：在整个频率范围内，$|Z_{PDN}(f)| < Z_{target}(f)$

### 7.2.5 阻抗优化算法

使用约束优化算法最小化PDN阻抗：

**目标函数**：
$$\min \max_{f \in [f_{min}, f_{max}]} |Z_{PDN}(f)|$$

**约束条件**：
1. 电容值约束：$C_i \in \{C_{available}\}$（标准值）
2. 数量约束：$\sum_{i} n_i \leq N_{max}$
3. 成本约束：$\sum_{i} n_i \cdot cost_i \leq Budget$
4. 空间约束：$\sum_{i} n_i \cdot area_i \leq Area_{available}$

**优化方法**：
- 遗传算法(GA)：适合离散优化问题
- 粒子群优化(PSO)：快速收敛
- 模拟退火(SA)：避免局部最优

## 7.3 去耦电容的选择与布局算法

### 7.3.1 电容的频率特性

实际电容并非理想元件，具有寄生参数：

```
    ESR        ESL
 ───/\/\/\───────────
           │
           C
           │
 ────────────────────
```

其阻抗为：
$$Z_C(f) = ESR + j2\pi f \cdot ESL + \frac{1}{j2\pi f \cdot C}$$

自谐振频率(SRF)：
$$f_{SRF} = \frac{1}{2\pi\sqrt{ESL \cdot C}}$$

在SRF处，电容阻抗最小：
$$Z_{min} = ESR$$

频率特性分为三个区域：
- $f < f_{SRF}$：容性区，$|Z| \propto 1/f$
- $f = f_{SRF}$：谐振点，$|Z| = ESR$
- $f > f_{SRF}$：感性区，$|Z| \propto f$

### 7.3.2 多级去耦策略

不同容值的电容覆盖不同频段：

| 电容类型 | 容值范围 | 有效频率 | 典型ESL | 功能 |
|---------|---------|---------|---------|------|
| 大电解电容 | 100-1000μF | DC-100kHz | 5-10nH | 低频储能 |
| 钽电容 | 10-100μF | 10kHz-1MHz | 2-5nH | 中低频滤波 |
| 陶瓷电容(1206) | 1-10μF | 100kHz-10MHz | 1-2nH | 中频去耦 |
| 陶瓷电容(0603) | 0.1-1μF | 1-100MHz | 0.5-1nH | 高频去耦 |
| 陶瓷电容(0402) | 1-100nF | 10-500MHz | 0.2-0.5nH | 超高频去耦 |

### 7.3.3 去耦电容数量计算

基于电荷平衡原理：

$$N_{cap} = \frac{I_{max} \cdot \Delta t}{C_{single} \cdot \Delta V_{allowed}}$$

其中：
- $I_{max}$ = 最大瞬态电流
- $\Delta t$ = 电流变化时间
- $C_{single}$ = 单个电容的容值
- $\Delta V_{allowed}$ = 允许的电压跌落

考虑并联电容的阻抗：
$$Z_{parallel} = \frac{Z_{single}}{N} = \frac{ESR + j\omega ESL + \frac{1}{j\omega C}}{N}$$

### 7.3.4 布局优化算法

去耦电容的布局直接影响其效果。关键参数是电容到负载的环路电感：

$$L_{loop} = L_{via} + L_{trace} + L_{plane}$$

**优化目标**：最小化加权环路电感
$$\min \sum_{i=1}^{N} w_i \cdot L_{loop,i}$$

其中权重$w_i$反映负载i的重要性。

**布局算法**：

1. **贪婪算法**：
   ```python
   for each capacitor:
       place at position minimizing L_loop
   ```

2. **K-means聚类**：
   - 将负载点聚类
   - 在每个簇中心放置去耦电容组

3. **模拟退火布局**：
   ```python
   T = T_initial
   while T > T_final:
       new_position = random_move(current_position)
       ΔE = cost(new_position) - cost(current_position)
       if ΔE < 0 or random() < exp(-ΔE/T):
           current_position = new_position
       T = T * cooling_rate
   ```

### 7.3.5 电容布局规则

1. **就近原则**：电容尽可能靠近负载引脚
   - 高频电容：< 5mm
   - 中频电容：< 20mm
   - 低频电容：< 50mm

2. **过孔优化**：
   - 使用短而粗的过孔（降低电感）
   - 并联多个过孔（电感反比于数量）
   - 过孔电感估算：
   $$L_{via} = \frac{\mu_0 h}{2\pi} \ln\left(\frac{d + \sqrt{d^2 + 4h^2}}{d}\right)$$
   
   其中h为过孔高度，d为直径

3. **平面耦合**：
   - 电容焊盘直接连接到电源/地平面
   - 避免长走线连接

### 7.3.6 反谐振问题与解决

多个不同容值的电容并联可能产生反谐振：

$$f_{anti} = \frac{1}{2\pi\sqrt{(ESL_1 + ESL_2) \cdot \frac{C_1 \cdot C_2}{C_1 + C_2}}}$$

在反谐振频率，阻抗出现峰值。解决方法：

1. **阻尼电阻**：串联小电阻增加阻尼
2. **容值递进**：相邻容值比< 10:1
3. **相同容值并联**：避免反谐振

## 7.4 同步开关噪声(SSN)分析

### 7.4.1 SSN的产生机理

同步开关噪声(Simultaneous Switching Noise)，也称为地弹(Ground Bounce)或ΔI噪声，是多个输出驱动器同时切换状态时产生的电源/地噪声。在现代高速数字系统中，SSN已成为限制I/O性能的主要因素。

#### 物理机理

当多个输出同时从低电平切换到高电平时，瞬态电流通过封装和PCB的寄生电感，产生感应电压：

$$V_{SSN} = L_{eff} \cdot \frac{di}{dt} \cdot N$$

其中：
- $L_{eff}$ = 有效回路电感（包括引线、过孔、平面）
- $di/dt$ = 单个输出的电流变化率
- $N$ = 同时开关的输出数量

对于CMOS驱动器，电流变化率取决于：
$$\frac{di}{dt} = \frac{I_{peak}}{t_r} = \frac{C_{load} \cdot V_{DD}}{t_r^2}$$

其中：
- $C_{load}$ = 负载电容
- $V_{DD}$ = 电源电压
- $t_r$ = 上升时间

### 7.4.2 SSN的数学建模

#### 集总参数模型

考虑N个同时开关的输出，共享电源/地回路：

```
     VDD
      │
   ┌──┴──┬──┬──┐
   │     │  │  │
   Z1    Z2 ... ZN  (输出驱动器)
   │     │  │  │
   └──┬──┴──┴──┘
      │
   L_gnd (共享地电感)
      │
     GND
```

总电流：$I_{total}(t) = \sum_{i=1}^{N} I_i(t)$

地弹电压：
$$V_{bounce} = L_{gnd} \cdot \frac{d I_{total}}{dt} + R_{gnd} \cdot I_{total}$$

#### 频域分析

在频域中，SSN可表示为：
$$V_{SSN}(f) = Z_{PDN}(f) \cdot I_{switch}(f) \cdot N_{eff}(f)$$

其中有效开关数量$N_{eff}(f)$考虑了相位关系：
$$N_{eff}(f) = \left|\sum_{i=1}^{N} e^{j\phi_i(f)}\right|$$

当所有输出完全同步时，$N_{eff} = N$（最坏情况）。

### 7.4.3 SSN的影响因素

#### 1. 封装参数的影响

封装电感是SSN的主要贡献者：

| 封装类型 | 典型电感 | SSN风险 |
|---------|---------|---------|
| DIP | 15-30 nH | 很高 |
| QFP | 5-15 nH | 高 |
| BGA | 1-5 nH | 中等 |
| CSP | 0.5-2 nH | 低 |
| Flip-Chip | 0.1-0.5 nH | 很低 |

#### 2. 信号边沿速率

SSN与边沿速率的关系：
$$V_{SSN} \propto \frac{1}{t_r}$$

控制边沿速率的方法：
- 驱动器强度控制
- 串联电阻
- 预加重/去加重

#### 3. 电源/地引脚比例

增加电源/地引脚可降低有效电感：
$$L_{eff} = \frac{L_{single}}{N_{P/G}}$$

其中$N_{P/G}$是并联的电源/地引脚数。

经验法则：电源/地引脚应占总引脚数的30-40%。

### 7.4.4 SSN的定量评估

#### SSN预算分析

总噪声预算：
$$V_{noise,total} = V_{SSN} + V_{crosstalk} + V_{reflection} + V_{PDN}$$

要求：$V_{noise,total} < V_{margin}$

其中噪声裕量：
$$V_{margin} = V_{OH} - V_{IH}$$（高电平）
$$V_{margin} = V_{IL} - V_{OL}$$（低电平）

#### SSN系数计算

定义SSN系数：
$$K_{SSN} = \frac{V_{bounce}}{N \cdot I_{peak}}$$

单位：mV/(mA·pin)

典型值：
- 良好设计：< 0.5 mV/(mA·pin)
- 可接受：0.5-1.0 mV/(mA·pin)
- 需改进：> 1.0 mV/(mA·pin)

### 7.4.5 SSN的测量方法

#### 1. 时域测量

使用示波器直接测量：
- 测试模式：所有输出同时翻转
- 测量点：靠近芯片的电源/地
- 带宽要求：> 5×(1/t_r)

#### 2. 频域测量

使用VNA测量PDN阻抗：
$$SSN_{potential} = |Z_{PDN}(f)| \cdot |I_{spectrum}(f)|$$

#### 3. 仿真预测

使用SPICE或专用PI工具：
```spice
* SSN仿真模型
.SUBCKT SSN_MODEL VDD VSS OUT
L_vdd VDD VDD_int 2n
L_vss VSS VSS_int 3n
* N个并联驱动器
X1 VDD_int VSS_int OUT1 DRIVER
X2 VDD_int VSS_int OUT2 DRIVER
...
XN VDD_int VSS_int OUTN DRIVER
.ENDS
```

### 7.4.6 SSN的抑制策略

#### 1. 差分信号

使用差分信号可大幅降低SSN：
- 电流方向相反，磁场相消
- 理论上SSN降低20-40dB

#### 2. 扩频时钟(SSC)

通过调制时钟频率，分散频谱能量：
$$f_{clk}(t) = f_0 \cdot [1 + \delta \cdot \sin(2\pi f_m t)]$$

其中：
- $\delta$ = 调制深度（典型0.5-2%）
- $f_m$ = 调制频率（典型30-33kHz）

SSN降低：
$$Reduction = 10\log_{10}\left(\frac{BW_{SSC}}{BW_{original}}\right)$$

#### 3. 相位错开

将输出分组，错开开关时间：
$$t_{group,i} = t_0 + i \cdot \Delta t$$

有效开关数量降低：
$$N_{eff} = \frac{N}{G}$$

其中G是分组数量。

#### 4. 嵌入式电容(Embedded Capacitance)

在PCB内嵌入薄介质层（<50μm）：
$$C_{embedded} = \epsilon_0 \epsilon_r \frac{A}{d}$$

优势：
- 超低ESL（< 50pH）
- 高频响应优异（> 1GHz）
- 节省表面空间

## 7.5 电源平面谐振与抑制策略

### 7.5.1 平面谐振的物理本质

电源/地平面对构成了一个二维谐振腔。当激励频率与谐振频率匹配时，会在平面内形成驻波，导致某些位置出现电压峰值（热点）和电压零点（冷点）。这种空间不均匀性会严重影响电源完整性。

#### 谐振模式分析

对于矩形平面（尺寸a×b），谐振频率由以下公式确定：

$$f_{mn} = \frac{c}{2\sqrt{\epsilon_r \mu_r}}\sqrt{\left(\frac{m}{a}\right)^2 + \left(\frac{n}{b}\right)^2}$$

其中(m,n)为模式编号，代表x和y方向的半波长数量。

主要模式特征：
- (1,0)模式：沿长边的半波谐振
- (0,1)模式：沿短边的半波谐振
- (1,1)模式：对角线谐振
- (2,0)、(0,2)模式：全波谐振

#### 电场分布

谐振时的电场分布：
$$E_z(x,y) = E_0 \sin\left(\frac{m\pi x}{a}\right) \sin\left(\frac{n\pi y}{b}\right)$$

电压分布：
$$V(x,y) = h \cdot E_z(x,y)$$

其中h为平面间距。

### 7.5.2 谐振的危害与表征

#### 1. 阻抗峰值

在谐振频率处，PDN阻抗急剧上升：
$$Z_{peak} = \frac{Q \cdot Z_0}{\pi}$$

其中：
- Q = 品质因数 = $\frac{f_r}{\Delta f_{3dB}}$
- $Z_0$ = 平面特性阻抗

高Q值意味着尖锐的谐振峰，可能导致：
- 局部电压崩溃
- EMI辐射增强
- 系统不稳定

#### 2. 空间不均匀性

定义电压变化系数：
$$\sigma_V = \frac{V_{max} - V_{min}}{V_{avg}}$$

谐振时，$\sigma_V$可达50%以上，意味着不同位置的芯片可能经历完全不同的电源条件。

#### 3. 耦合效应

平面谐振会耦合到信号线，造成串扰：
$$S_{21,resonance} = 20\log_{10}\left(\frac{Z_{mutual}}{Z_0}\right)$$

在谐振频率，耦合可增强20-30dB。

### 7.5.3 谐振抑制技术

#### 1. 电磁带隙结构(EBG)

EBG是一种周期性结构，在特定频段内阻止电磁波传播：

```
┌─┬─┬─┬─┬─┐
├─┼─┼─┼─┼─┤  周期性贴片
├─┼─┼─┼─┼─┤
├─┼─┼─┼─┼─┤
└─┴─┴─┴─┴─┘
```

带隙频率：
$$f_{gap} = \frac{1}{2\pi\sqrt{L_{unit} \cdot C_{unit}}}$$

其中：
- $L_{unit}$ = 单元电感（过孔电感）
- $C_{unit}$ = 单元电容（贴片电容）

设计参数：
- 贴片尺寸：$a_{patch} = \frac{\lambda_{gap}}{4\sqrt{\epsilon_r}}$
- 间隙宽度：影响带隙宽度
- 过孔直径：影响电感值

#### 2. 去耦电容的策略性布局

基于模式分析的电容布局：

**反节点布局**：将电容放置在电压最大处
$$x_{cap} = \frac{a}{2m}(2k-1), \quad k=1,2,...,m$$
$$y_{cap} = \frac{b}{2n}(2j-1), \quad j=1,2,...,n$$

**多模式抑制**：针对不同谐振模式使用不同SRF的电容
- 低频模式：大容值电容（10-100μF）
- 中频模式：中等容值（0.1-1μF）
- 高频模式：小容值（1-100nF）

#### 3. 有损边界

在平面边缘添加电阻性材料，增加损耗：

$$\alpha_{edge} = \frac{R_{edge}}{2Z_0}$$

其中$R_{edge}$是边缘电阻（Ω/square）。

实现方法：
- 电阻性胶带
- 印刷电阻膏
- 分布式贴片电阻

损耗与Q值的关系：
$$Q_{loaded} = \frac{Q_{unloaded}}{1 + Q_{unloaded} \cdot \alpha_{edge}}$$

#### 4. 平面分割与隔离

将大平面分割成小区域，提高最低谐振频率：

$$f_{min,segmented} = n \cdot f_{min,original}$$

其中n是分割系数。

分割策略：
- 功能分区：模拟/数字/RF分离
- 星形连接：单点接地
- 护城河隔离：围绕敏感区域

### 7.5.4 谐振预测与仿真

#### 1. 解析方法

使用格林函数求解：
$$G(x,y;x',y') = \sum_{m=0}^{\infty}\sum_{n=0}^{\infty} \frac{4\epsilon_m\epsilon_n}{ab} \cdot \frac{\sin(k_mx)\sin(k_mx')\sin(k_ny)\sin(k_ny')}{k^2 - k_{mn}^2}$$

其中：
- $\epsilon_m, \epsilon_n$ = Neumann因子（m=0时为1，否则为2）
- $k_{mn}^2 = (m\pi/a)^2 + (n\pi/b)^2$

#### 2. 数值方法

**有限元法(FEM)**：
- 网格密度：$\lambda_{min}/20$
- 边界条件：PEC或PMC
- 求解器：频域或本征模

**时域有限差分(FDTD)**：
- 空间步长：$\Delta x < \lambda_{min}/20$
- 时间步长：$\Delta t < \Delta x/(c\sqrt{3})$（CFL条件）
- 激励：高斯脉冲或调制高斯

#### 3. 测量验证

使用近场扫描系统：
- 探头类型：电场或磁场探头
- 扫描分辨率：< λ/10
- 频率范围：10MHz - 6GHz

测量指标：
- S21（传输系数）
- 阻抗分布图
- 电场强度分布

### 7.5.5 先进抑制技术

#### 1. 超材料吸收器

设计具有负介电常数或负磁导率的人工结构：

$$\epsilon_{eff} = \epsilon_0\left(1 - \frac{f_p^2}{f^2 + j\gamma f}\right)$$

其中$f_p$是等离子频率，$\gamma$是损耗因子。

#### 2. 主动噪声消除

使用反馈控制系统主动注入反相电流：

```
Noise → Sensor → Controller → Actuator → -Noise
         ↑                              ↓
         └──────── Feedback ────────────┘
```

控制律：
$$I_{cancel}(s) = -G(s) \cdot V_{noise}(s)$$

其中G(s)是控制器传递函数。

#### 3. 自适应阻抗控制

根据工作状态动态调整PDN阻抗：
- 低功耗模式：增加阻抗，降低静态功耗
- 高性能模式：降低阻抗，提供更好的瞬态响应

实现方式：
- 可变电容阵列
- 开关电容网络
- 数字控制DC-DC转换器
