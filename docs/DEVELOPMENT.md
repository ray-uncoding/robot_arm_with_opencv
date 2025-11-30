# 🔧 專案開發說明

## 系統架構

### 核心模組

#### 1. integrated_controller.py
**主控制程式**，整合所有功能的 PyQt5 應用程式。

**主要類別：**
- `RobotArmController`: 主視窗類別
  - 管理 Serial 通訊
  - 整合三個標籤頁（運動學、執行、視覺）
  - 處理即時控制與軌跡執行

**關鍵方法：**
- `connect_arduino()`: 建立 Serial 連接
- `send_angles_to_arduino()`: 發送角度命令
- `enhanced_update_display()`: 即時更新顯示（延遲 300ms 防抖）
- `start_trajectory_execution()`: 執行軌跡（500ms 間隔）
- `update_camera_frame()`: 更新視覺監控畫面（30ms 間隔）

#### 2. kinematics_control.py
**運動學計算模組**，處理 DH 參數、正逆向運動學。

**主要功能：**
- `get_dh_params()`: 計算 DH 參數（包含軸 2、4 方向反轉）
- `forward_kinematics()`: 正向運動學計算末端位置
- `inverse_kinematics_iterative()`: 逆向運動學（混合演算法）
  - 網格搜尋階段：30° 間隔粗搜
  - 梯度下降階段：2° 步進精搜
- `plan_trajectory()`: 線性插值生成軌跡
- `create_animation_window()`: 建立 3D 視覺化動畫

**DH 參數映射：**
```python
# 軸 2 和軸 4 需要方向反轉
servo_angles = [theta1, theta2, theta3, theta4]
dh_angles = [
    theta1,           # 軸 1: 正向
    180 - theta2,     # 軸 2: 反向
    theta3,           # 軸 3: 正向
    180 - theta4,     # 軸 4: 反向
]
```

#### 3. robot_arm_with_opencv.ino
**Arduino 韌體**，控制 PCA9685 伺服驅動板。

**功能：**
- 解析 Serial 命令：`angle1,angle2,angle3,angle4\n`
- 驅動 4 個伺服馬達
- 回傳執行狀態

## 資料流程

### 即時控制流程
```
[使用者拖曳滑桿] 
    ↓
[valueChanged 信號]
    ↓
[300ms 延遲計時器]
    ↓
[send_angles_to_arduino()]
    ↓
[Serial 發送]
    ↓
[Arduino 執行]
    ↓
[伺服馬達移動]
```

### 軌跡執行流程
```
[規劃路徑] 
    ↓
[生成航點列表]
    ↓
[開始執行]
    ↓
[500ms QTimer]
    ↓
[依序發送每個航點]
    ↓
[更新進度條與日誌]
    ↓
[執行完成]
```

### 視覺監控流程
```
[開啟攝影機] 
    ↓
[30ms QTimer]
    ↓
[讀取影像幀]
    ↓
[BGR → HSV 轉換]
    ↓
[HSV 閾值過濾]
    ↓
[形態學處理]
    ↓
[輪廓檢測]
    ↓
[標記紅點]
    ↓
[顯示於 QLabel]
```

## 重要參數

### 伺服控制
```python
SERVO_MIN = 20    # 最小角度
SERVO_MAX = 160   # 最大角度
SERVO_INIT = 90   # 初始位置
```

### 通訊設定
```python
BAUD_RATE = 115200
TIMEOUT = 1.0  # 秒
```

### 定時器間隔
```python
SLIDER_DELAY = 300   # 滑桿防抖延遲 (ms)
TRAJECTORY_INTERVAL = 500  # 軌跡執行間隔 (ms)
CAMERA_UPDATE = 30   # 攝影機更新間隔 (ms)
```

### HSV 預設值（紅色檢測）
```python
H_MIN = 0    # 色相下限
H_MAX = 10   # 色相上限
S_MIN = 100  # 飽和度下限
S_MAX = 255  # 飽和度上限
V_MIN = 100  # 亮度下限
V_MAX = 255  # 亮度上限
```

## 已知問題與解決方案

### ✅ 已解決

1. **逆運動學精度不足**
   - 問題：Newton-Raphson 方法完全失效
   - 解決：改用混合式網格搜尋 + 梯度下降
   - 結果：< 5mm 誤差，100% 成功率

2. **軸向反轉問題**
   - 問題：軸 2、4 移動方向與 DH 參數不符
   - 解決：在 `get_dh_params()` 中加入反轉映射
   - 結果：所有軸向正確

3. **軌跡執行按鈕失效**
   - 問題：軌跡未同步，按鈕被禁用
   - 解決：自動同步 + 手動「載入」按鈕
   - 結果：執行正常

4. **3D 動畫警告**
   - 問題：動畫物件被垃圾回收
   - 解決：儲存為成員變數 `self.animation`
   - 結果：警告消失

### ⚠️ 待改進

1. **工作空間邊界檢查**
   - 建議：在逆運動學前檢查目標是否在可達範圍
   
2. **碰撞檢測**
   - 建議：加入自碰撞檢測機制

3. **速度控制**
   - 建議：加入關節速度限制與平滑加減速

## 測試指令

### 測試正向運動學
```bash
cd tests
python debug_fk.py
```

### 測試逆向運動學
```bash
cd tests
python test_simple_ik.py
```

### 運動學分析
```bash
cd tests
python analyze_kinematics.py
```

### 完整測試
```bash
cd tests
python test_kinematics.py
```

## 除錯技巧

### 啟用詳細日誌
在 `integrated_controller.py` 中：
```python
def log_message(self, message):
    timestamp = datetime.now().strftime("%H:%M:%S")
    print(f"[{timestamp}] {message}")  # 添加 print 輸出
    if hasattr(self, 'log_text'):
        self.log_text.append(f"[{timestamp}] {message}")
```

### 檢查 Serial 通訊
使用 Arduino Serial Monitor 或：
```python
# 在 send_angles_to_arduino() 中加入
print(f"Sending: {command}")
```

### 視覺化 DH 參數
```python
dh_params = self.kinematics.get_dh_params(angles)
print("DH Parameters:")
print(dh_params)
```

## 擴充功能建議

### 1. 力回饋
- 整合力矩感測器
- 實現阻抗控制

### 2. 路徑規劃優化
- 改用 B-spline 或 NURBS 曲線
- 加入關節速度最佳化

### 3. 機器學習
- 使用學習的逆運動學模型
- 手勢辨識控制

### 4. 多機協同
- 支援多台機械手臂
- 協同作業規劃

### 5. 視覺伺服
- 物體識別與追蹤
- 視覺導引定位

## 貢獻指南

1. Fork 專案
2. 建立功能分支
3. 提交變更
4. 發送 Pull Request

## 聯絡方式

如有問題或建議，請建立 Issue。
