<div class="cover-container">
  <div class="cover-content">
    <img src="logo.svg" alt="电机控制" class="cover-logo">
    <h1>电机控制文档中心 <small>v1.0.0</small></h1>
    <p class="cover-description">
      专业的电机控制技术文档库，涵盖BLDC、FOC等控制算法<br>
      以及ADC、TIM等硬件模块的应用指南
    </p>
    
    <div class="cover-buttons">
      <a href="/motorcontrol/基础知识.md" class="btn btn-primary">开始学习</a>
      <a href="/motorcontrol/FOC电控.md" class="btn btn-secondary">FOC控制</a>
    </div>
    
    <div class="cover-footer">
      STM32 | 电机驱动 | 控制算法 | 硬件设计
    </div>
  </div>
</div>

<style>
.cover-container {
  display: flex;
  justify-content: center;
  align-items: center;
  min-height: 100vh;
  text-align: center;
  padding: 20px;
}

.cover-content {
  max-width: 800px;
  margin: 0 auto;
}

.cover-logo {
  height: 120px;
  margin-bottom: 30px;
  display: block;
}

.cover-description {
  color: #666;
  margin: 20px 0;
  line-height: 1.6;
  font-size: 18px;
}

.cover-buttons {
  margin: 40px 0;
  display: flex;
  justify-content: center;
  gap: 15px;
}

.btn {
  padding: 12px 24px;
  border-radius: 4px;
  text-decoration: none;
  font-weight: bold;
  transition: all 0.3s;
}

.btn-primary {
  background: #2c3e50;
  color: white;
}

.btn-secondary {
  background: #3498db;
  color: white;
}

.cover-footer {
  color: #7f8c8d;
  font-size: 14px;
  margin-top: 40px;
}
</style>
