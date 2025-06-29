# PID算法

## 一、什么是 PID？

**PID** 是比例-积分-微分（Proportional–Integral–Derivative）控制器的简称，是一种经典的闭环控制算法。它的输出由三个部分组成：

1. **P：比例项（Proportional）**
   - 根据当前位置误差（目标值 - 实际值）直接输出控制量。
   - 提高响应速度，误差越大输出越大。
   - 但单独使用会导致稳态误差。
2. **I：积分项（Integral）**
   - 对误差进行累积，长期误差越大，控制量越大。
   - 消除稳态误差。
   - 缺点：容易引起震荡或积分饱和。
3. **D：微分项（Derivative）**
   - 根据误差变化率调整控制量，预测趋势。
   - 提高系统的稳定性，减缓超调。
   - 对噪声敏感。

------

## 二、PID 控制器的数学表达式（连续域）

控制量 u(t)u(t) 表达为：

$u(t) = K_p \cdot e(t) + K_i \cdot \int_0^t e(\tau) d\tau + K_d \cdot \frac{de(t)}{dt}$

其中：

- $e(t) = r(t) - y(t)$：目标值减去实际值，称为**误差**；
- $K_p, K_i, K_d$：分别是比例、积分、微分系数。

------

## 三、位置环中的 PID 控制

**位置环控制**用于控制电机或执行器的位置，让其到达并保持在某个目标位置。

### PID 在位置环中的作用：

- **误差来源**：目标位置−当前实际位置目标位置 - 当前实际位置
- **PID 控制器输出**：通常是速度指令或力矩指令（再由下一级速度环或电流环来处理）
- **结构一般如下**：

```text
目标位置 ─┐
          ▼
      [误差计算] ──► [PID控制器] ──► 输出速度或电流指令
                      ▲
                    实际位置（来自编码器）
```

### 特点：

- 控制对象是**位移**，积分项尤为重要，可以消除稳态误差；
- D 项（微分）可以抑制快速误差变化，比如防止目标突变导致超调；
- 若位置控制链路中含有噪声，应谨慎使用 D 项。

------

## 四、离散 PID 实现公式（在嵌入式中常用）

假设采样周期为 T，使用离散误差 $e[n]$，常见实现方式有两种：

### 1. **位置式 PID**

每次计算完整的控制量：

$u[n] = K_p e[n] + K_i \sum_{i=0}^{n} e[i] \cdot T + K_d \cdot \frac{e[n] - e[n-1]}{T}$

缺点：积分容易超调，需要防止积分饱和。

### 2. **增量式 PID**

只计算本次相对上次的增量，更稳定、常用于 MCU 中：

$\Delta u[n] = K_p (e[n] - e[n-1]) + K_i e[n] \cdot T + K_d \cdot \frac{e[n] - 2e[n-1] + e[n-2]}{T}$

更新控制量：

$u[n] = u[n-1] + \Delta u[n]$

------

## 五、位置环 PID 调参建议

调参顺序建议如下：

1. **先调 P（比例）**：
   - 增加响应速度；
   - 过大会引起震荡或超调。
2. **再加 I（积分）**：
   - 慢慢增加消除稳态误差；
   - 太大会造成震荡甚至积分饱和。
3. **最后加 D（微分）**：
   - 增强系统稳定性，减缓超调；
   - 有噪声时建议使用带低通滤波的微分项。

------

## 六、示例代码（C语言伪代码）

```c
typedef struct {
    float Kp, Ki, Kd;
    float err, err_last, err_prev;
    float integral;
    float output;
} PID_t;

float PID_Position_Calc(PID_t *pid, float target, float current) {
    pid->err = target - current;
    pid->integral += pid->err;
    float derivative = pid->err - pid->err_last;
    
    pid->output = pid->Kp * pid->err 
                + pid->Ki * pid->integral //这里应该少了Ts
                + pid->Kd * (pid->err - 2*pid->err_last + pid->err_prev);//这里应该少了Ts
    
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err;
    return pid->output;
}

///方案1：标准位置式PID（推荐）
typedef struct {
    float Kp, Ki, Kd;  // PID参数
    float Ts;           // 采样时间
    float integral;     // 积分累积值
    float prev_err;     // 上一次误差
} PID;

float PID_Update(PID* pid, float current, float target) {
    // 计算当前误差
    float err = target - current;
    
    // 积分项累积（防饱和处理可选）
    pid->integral += err * pid->Ts;
    
    // 微分项 = (当前误差 - 上次误差) / 采样时间
    float derivative = (err - pid->prev_err) / pid->Ts;
    
    // PID输出
    pid->output = pid->Kp * err 
                + pid->Ki * pid->integral 
                + pid->Kd * derivative;
    
    // 更新历史误差
    pid->prev_err = err;
    
    return pid->output;
}

////方案2：增量式PID（适合执行器）
float PID_Update(PID* pid, float current, float target) {
    float err = target - current;
    
    // 计算各分量
    float P = pid->Kp * (err - pid->err_last);
    float I = pid->Ki * err * pid->Ts;
    float D = pid->Kd * (err - 2*pid->err_last + pid->err_prev) / pid->Ts;
    
    // 输出增量
    float delta_output = P + I + D;
    
    // 更新误差历史
    pid->err_prev = pid->err_last;
    pid->err_last = err;
    
    return delta_output;  // 注意：外部需累加此增量
}
```
