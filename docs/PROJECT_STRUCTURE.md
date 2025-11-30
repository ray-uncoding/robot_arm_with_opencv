# 📂 專案結構總覽

```
robot_arm_with_opencv/
│
├── 📄 README.md                      # 專案主要說明文件
├── 📄 .gitignore                     # Git 忽略檔案設定
├── 📄 requirements.txt               # Python 依賴套件清單
│
├── 🐍 integrated_controller.py       # ⭐ 主控制程式
├── 🐍 kinematics_control.py          # ⭐ 運動學模組
├── 🔌 robot_arm_with_opencv.ino      # ⭐ Arduino 韌體
│
├── 🚀 start_controller.bat           # Windows 啟動腳本
├── 🚀 start_robot_system.bat         # 系統啟動腳本
│
├── 📚 docs/                          # 文件資料夾
│   ├── DEVELOPMENT.md                # 開發說明文件
│   └── KINEMATICS_README.md          # 運動學詳細說明
│
├── 🧪 tests/                         # 測試與除錯工具
│   ├── test_kinematics.py            # 運動學完整測試
│   ├── test_simple_ik.py             # 簡易逆運動學測試
│   ├── analyze_kinematics.py         # 運動學分析工具
│   ├── debug_fk.py                   # 正運動學除錯
│   ├── 01.json                       # 測試軌跡檔案 1
│   └── test.json                     # 測試軌跡檔案 2
│
└── 📦 legacy/                        # 舊版程式碼
    ├── robot_arm_controller.py       # 第一版控制器
    └── kinematics_requirements.txt   # 舊版依賴清單
```

## 核心檔案說明

### 主程式檔案

#### `integrated_controller.py` (770+ 行)
**完整的 PyQt5 控制介面**

- 三個主要標籤頁：
  - 🎯 運動學控制：即時控制、航點記錄、軌跡規劃、3D 視覺化
  - ▶️ 軌跡執行：載入/執行軌跡、進度顯示
  - 📷 視覺監控：攝影機監控、紅點檢測、HSV 調整

- 核心功能：
  - Serial 通訊管理
  - 即時角度控制（300ms 防抖）
  - 軌跡執行系統（500ms 間隔）
  - 視覺監控（30ms 更新）

#### `kinematics_control.py` (400+ 行)
**運動學計算核心**

- DH 參數建模（5 關節）
- 正向運動學（齊次轉換矩陣）
- 逆向運動學（混合式演算法）
  - 網格搜尋：30° 間隔
  - 梯度下降：2° 步進
- 軌跡規劃（線性插值）
- 3D 視覺化（matplotlib 動畫）

#### `robot_arm_with_opencv.ino` (Arduino)
**嵌入式控制韌體**

- PCA9685 I2C 伺服驅動
- Serial 命令解析
- 4 軸伺服控制
- 狀態回報

### 配置檔案

#### `requirements.txt`
```
pyqt5>=5.15.0          # GUI 框架
pyserial>=3.5          # Serial 通訊
numpy>=1.21.0          # 數值計算
scipy>=1.8.0           # 科學計算
matplotlib>=3.5.0      # 繪圖與 3D 視覺化
opencv-python>=4.5.0   # 視覺監控
```

### 文件檔案

#### `README.md`
- ✨ 功能特色介紹
- 🚀 快速開始指南
- 📖 使用教學
- 🎯 DH 參數表
- 🐛 故障排除
- 📝 更新日誌

#### `docs/DEVELOPMENT.md`
- 🏗️ 系統架構說明
- 🔄 資料流程圖
- ⚙️ 重要參數設定
- ✅ 已解決問題記錄
- 🧪 測試指令
- 🔧 除錯技巧
- 💡 擴充功能建議

#### `docs/KINEMATICS_README.md`
- 📐 DH 參數理論
- 🔢 數學公式推導
- 📊 運動學計算範例

## 測試工具說明

### `test_kinematics.py`
完整的運動學系統測試

- 正向運動學測試
- 逆向運動學測試
- 軌跡規劃測試
- 邊界條件測試

### `test_simple_ik.py`
簡化的逆運動學測試

- 單點求解測試
- 精度驗證
- 收斂性分析

### `analyze_kinematics.py`
運動學分析工具

- 工作空間分析
- 可達性檢查
- 奇異點檢測

### `debug_fk.py`
正向運動學除錯

- DH 參數檢查
- 轉換矩陣驗證
- 末端位置計算

## 軌跡檔案格式

### JSON 結構
```json
{
  "waypoints": [
    {
      "theta1": 90,
      "theta2": 90,
      "theta3": 90,
      "theta4": 90
    },
    {
      "theta1": 45,
      "theta2": 60,
      "theta3": 120,
      "theta4": 90
    }
  ]
}
```

### 檔案位置
- `tests/01.json` - 測試軌跡 1
- `tests/test.json` - 測試軌跡 2
- 使用者自定義軌跡可儲存於任意位置

## 舊版檔案（legacy/）

### `robot_arm_controller.py`
第一版控制器，僅包含基礎功能：
- Serial 通訊
- 簡單滑桿控制
- 無運動學計算
- 無視覺監控

**保留原因：** 參考用途，展示專案演進歷程

## 檔案大小參考

```
integrated_controller.py    ~35 KB
kinematics_control.py       ~18 KB
robot_arm_with_opencv.ino   ~3 KB
README.md                   ~12 KB
DEVELOPMENT.md              ~8 KB
```

## 程式碼統計

```
總計：
- Python 程式碼：~1200 行
- Arduino 程式碼：~100 行
- 文件：~500 行
- 測試程式碼：~400 行
```

## 快速導航

### 想要...

**啟動程式** → `start_controller.bat` 或 `python integrated_controller.py`

**了解功能** → `README.md`

**開發擴充** → `docs/DEVELOPMENT.md`

**學習運動學** → `docs/KINEMATICS_README.md`

**測試系統** → `tests/test_kinematics.py`

**除錯問題** → `tests/debug_fk.py`

**查看舊版** → `legacy/robot_arm_controller.py`

## 建議閱讀順序

1. 📄 **README.md** - 了解專案概況
2. 🚀 **快速開始** - 執行程式
3. 📖 **使用指南** - 學習操作
4. 📚 **DEVELOPMENT.md** - 深入架構
5. 📐 **KINEMATICS_README.md** - 理解運動學
6. 🧪 **執行測試** - 驗證功能

---

最後更新：2025-12-01
