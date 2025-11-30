#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
機械手臂運動學控制系統
包含正向運動學、逆向運動學、路徑規劃和3D動畫模擬
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
import json
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLabel, QSlider, QPushButton, QComboBox, 
                             QGroupBox, QTableWidget, QTableWidgetItem, QSpinBox, 
                             QDoubleSpinBox, QTextEdit, QTabWidget, QGridLayout,
                             QMessageBox, QFileDialog, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QFont
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class DHParameters:
    """DH參數類別 - 可自行修改"""
    def __init__(self):
        # DH參數表格 [theta, d, a, alpha] (單位: 度, mm, mm, 度)
        # 請根據你的實際機械手臂規格修改這些數值
        self.dh_table = np.array([
            [0,    30,   0,   -90],     # 關節1: 底座旋轉
            [-90,  0,    160,   0],     # 關節2: 大臂
            [90,   0,    140,   0],     # 關節3: 小臂  
            [90,   0,    120,    0],     # 關節4: 夾爪軸
            [-90,  0,     55,    0]       # 末端校正,不能轉，僅代表末端位置
        ])
        
        # 關節限制 [min, max] (度) - 根據硬體限制
        self.joint_limits = np.array([
            [20, 160],    # 關節1: 底座旋轉 (20-160度)
            [20, 160],    # 關節2: 大臂 (20-160度)
            [20, 160],    # 關節3: 小臂 (20-160度)
            [20, 160]     # 關節4: 夾爪 (20-160度)
        ])
        
    def get_dh_params(self, joint_angles):
        """獲取當前關節角度的DH參數"""
        dh_current = self.dh_table.copy()
        # joint_angles是伺服馬達的絕對角度，需要轉換為DH參數的偏移
        if len(joint_angles) == 4:
            for i in range(4):
                # DH theta = 初始 theta + (伺服角度 - 初始伺服角度)
                servo_offset = joint_angles[i] - 90  # 90是伺服馬達初始位置
                
                # 第二軸 (大臂) 和第四軸 (夾爪) 需要反向映射
                if i == 1 or i == 3:  # 第二軸和第四軸
                    servo_offset = -servo_offset
                    
                dh_current[i, 0] = self.dh_table[i, 0] + servo_offset
        return dh_current
    
    def deg_to_rad(self, angles):
        """角度轉弧度"""
        return np.deg2rad(angles)
    
    def rad_to_deg(self, angles):
        """弧度轉角度"""
        return np.rad2deg(angles)

class ForwardKinematics:
    """正向運動學類別"""
    def __init__(self, dh_params):
        self.dh_params = dh_params
        
    def dh_transform_matrix(self, theta, d, a, alpha):
        """計算單一DH變換矩陣"""
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        T = np.array([
            [ct,    -st*ca,    st*sa,     a*ct],
            [st,    ct*ca,     -ct*sa,    a*st],
            [0,     sa,        ca,        d],
            [0,     0,         0,         1]
        ])
        return T
    
    def forward_kinematics(self, joint_angles):
        """計算正向運動學"""
        dh_current = self.dh_params.get_dh_params(joint_angles)
        
        # 初始變換矩陣
        T_total = np.eye(4)
        transforms = [T_total.copy()]
        
        # 逐個計算每個關節的變換
        for i in range(len(dh_current)):
            theta, d, a, alpha = dh_current[i]
            T_i = self.dh_transform_matrix(theta, d, a, alpha)
            T_total = T_total @ T_i
            transforms.append(T_total.copy())
            
        # 提取末端效應器位置和姿態
        end_position = T_total[:3, 3]
        end_orientation = T_total[:3, :3]
        
        return end_position, end_orientation, transforms
    
    def get_all_joint_positions(self, joint_angles):
        """獲取所有關節的位置"""
        _, _, transforms = self.forward_kinematics(joint_angles)
        positions = []
        
        for T in transforms:
            pos = T[:3, 3]
            positions.append(pos)
            
        return np.array(positions)

class InverseKinematics:
    """逆向運動學類別"""
    def __init__(self, dh_params, forward_kin):
        self.dh_params = dh_params
        self.forward_kin = forward_kin
        
    def jacobian(self, joint_angles):
        """計算雅可比矩陣 - 只考慮前4個可控關節"""
        epsilon = 1.0  # 使用更大的變化量(1度)
        # 確保只使用4個關節角度
        joint_angles = joint_angles[:4] if len(joint_angles) > 4 else joint_angles
        jacobian = np.zeros((3, len(joint_angles)))
        
        # 當前末端效應器位置
        current_pos, _, _ = self.forward_kin.forward_kinematics(joint_angles)
        
        for i in range(len(joint_angles)):
            # 正向微小變化
            angles_plus = joint_angles.copy()
            angles_plus[i] += epsilon
            # 確保在範圍內
            angles_plus[i] = np.clip(angles_plus[i], 20, 160)
            
            # 負向微小變化
            angles_minus = joint_angles.copy()
            angles_minus[i] -= epsilon
            # 確保在範圍內
            angles_minus[i] = np.clip(angles_minus[i], 20, 160)
            
            # 計算位置變化 (中央差分)
            pos_plus, _, _ = self.forward_kin.forward_kinematics(angles_plus)
            pos_minus, _, _ = self.forward_kin.forward_kinematics(angles_minus)
            
            # 實際的角度變化
            actual_delta = angles_plus[i] - angles_minus[i]
            
            # 計算偏導數 (位置變化 / 角度變化)
            if actual_delta > 0:
                jacobian[:, i] = (pos_plus - pos_minus) / actual_delta
            else:
                # 如果無法計算，使用單邊差分
                jacobian[:, i] = (pos_plus - current_pos) / epsilon
            
        return jacobian
    
    def inverse_kinematics_iterative(self, target_position, initial_angles=None, max_iterations=100, tolerance=5.0):
        """使用改進的數值方法求逆向運動學"""
        print(f"[IK] 目標位置: {target_position}")
        
        x, y, z = target_position
        
        # 第一關節(底座)可以直接計算
        theta1_rad = np.arctan2(y, x)
        servo1 = np.rad2deg(theta1_rad) + 90  # 轉換為伺服角度
        servo1 = np.clip(servo1, 20, 160)
        
        # 對於第2、3、4關節，使用網格搜索找到最佳組合
        best_angles = None
        best_error = float('inf')
        
        # 粗搜索
        for s2 in range(30, 151, 30):  # 第二關節: 30, 60, 90, 120, 150
            for s3 in range(30, 151, 30):  # 第三關節: 30, 60, 90, 120, 150
                for s4 in range(60, 121, 30):  # 第四關節: 60, 90, 120
                    test_angles = [servo1, s2, s3, s4]
                    test_pos, _, _ = self.forward_kin.forward_kinematics(test_angles)
                    error = np.linalg.norm(test_pos - target_position)
                    
                    if error < best_error:
                        best_error = error
                        best_angles = test_angles.copy()
        
        if best_angles is None:
            print(f"[IK] 網格搜索失敗")
            return np.array([90, 90, 90, 90]), False, 0
        
        # 精細搜索：在最佳點附近小範圍優化
        current_angles = np.array(best_angles, dtype=float)
        
        for iteration in range(50):
            current_pos, _, _ = self.forward_kin.forward_kinematics(current_angles)
            error_vec = target_position - current_pos
            error = np.linalg.norm(error_vec)
            
            if error < tolerance:
                print(f"[IK] 成功! 誤差: {error:.1f}mm, 角度: {current_angles}")
                return current_angles, True, iteration
            
            # 簡單的梯度下降
            # 計算每個關節微小變化對位置的影響
            step_size = 2.0  # 每次2度
            improved = False
            
            for i in [1, 2, 3]:  # 只優化234關節
                # 嘗試增加
                test_angles = current_angles.copy()
                test_angles[i] = np.clip(test_angles[i] + step_size, 20, 160)
                test_pos, _, _ = self.forward_kin.forward_kinematics(test_angles)
                error_plus = np.linalg.norm(test_pos - target_position)
                
                # 嘗試減少
                test_angles = current_angles.copy()
                test_angles[i] = np.clip(test_angles[i] - step_size, 20, 160)
                test_pos, _, _ = self.forward_kin.forward_kinematics(test_angles)
                error_minus = np.linalg.norm(test_pos - target_position)
                
                # 選擇更好的方向
                if error_plus < error and error_plus < error_minus:
                    current_angles[i] = np.clip(current_angles[i] + step_size, 20, 160)
                    improved = True
                elif error_minus < error:
                    current_angles[i] = np.clip(current_angles[i] - step_size, 20, 160)
                    improved = True
            
            if not improved:
                # 減小步長重試
                step_size *= 0.5
                if step_size < 0.1:
                    break
        
        # 最終檢查
        final_pos, _, _ = self.forward_kin.forward_kinematics(current_angles)
        final_error = np.linalg.norm(final_pos - target_position)
        success = final_error < tolerance * 2  # 放寬誤差容忍
        
        print(f"[IK] 完成! 誤差: {final_error:.1f}mm, 角度: {current_angles}, 成功: {success}")
        return current_angles, success, iteration

class PathPlanner:
    """路徑規劃類別"""
    def __init__(self, forward_kin, inverse_kin):
        self.forward_kin = forward_kin
        self.inverse_kin = inverse_kin
        
    def interpolate_path(self, start_pos, end_pos, num_points=5):
        """在兩點間插值生成路徑點"""
        path_points = []
        for i in range(num_points):
            t = i / (num_points - 1)
            interpolated_pos = start_pos + t * (end_pos - start_pos)
            path_points.append(interpolated_pos)
        return np.array(path_points)
    
    def generate_trajectory(self, waypoints, current_angles=None):
        """生成完整軌跡"""
        if len(waypoints) < 2:
            return [], []
        
        trajectory_points = []
        trajectory_angles = []
        
        # 起始角度 (伺服馬達角度)
        if current_angles is None:
            current_angles = np.array([90, 90, 90, 90])
        
        for i in range(len(waypoints) - 1):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]
            
            # 在兩個點位間插值
            path_points = self.interpolate_path(start_pos, end_pos, num_points=4)
            
            for point in path_points:
                trajectory_points.append(point)
                
                # 計算逆向運動學
                angles, success, iterations = self.inverse_kin.inverse_kinematics_iterative(
                    point, initial_angles=current_angles
                )
                
                if success:
                    trajectory_angles.append(angles)
                    current_angles = angles  # 更新當前角度
                else:
                    print(f"警告: 無法到達點位 {point}, 使用前一步角度")
                    # 使用前一步的角度而不是初始角度
                    trajectory_angles.append(current_angles.copy())
        
        return trajectory_points, trajectory_angles

class RobotVisualizer:
    """機械手臂3D視覺化類別"""
    def __init__(self, forward_kin):
        self.forward_kin = forward_kin
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.setup_plot()
        
    def setup_plot(self):
        """設置繪圖環境"""
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('機械手臂3D模擬')
        
        # 設置座標軸範圍
        limit = 400
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([0, limit])
        
    def draw_robot(self, joint_angles, waypoints=None, trajectory=None):
        """繪製機械手臂"""
        self.ax.clear()
        self.setup_plot()
        
        # 獲取所有關節位置
        positions = self.forward_kin.get_all_joint_positions(joint_angles)
        
        # 繪製手臂鏈條
        self.ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                    'b-o', linewidth=3, markersize=8, label='機械手臂')
        
        # 繪製關節
        for i, pos in enumerate(positions):
            self.ax.scatter(pos[0], pos[1], pos[2], 
                          s=100, c='red' if i == len(positions)-1 else 'blue')
            joint_name = f'END' if i == len(positions)-1 else f'J{i}'
            self.ax.text(pos[0], pos[1], pos[2]+10, joint_name, fontsize=8)
        
        # 繪製座標原點
        self.ax.scatter(0, 0, 0, s=200, c='green', marker='s', label='基座')
        
        # 繪製路徑點
        if waypoints is not None and len(waypoints) > 0:
            waypoints = np.array(waypoints)
            self.ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 
                          s=100, c='yellow', marker='^', label='目標點')
        
        # 繪製軌跡
        if trajectory is not None and len(trajectory) > 0:
            trajectory = np.array(trajectory)
            self.ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 
                        'g--', linewidth=2, alpha=0.7, label='規劃軌跡')
        
        self.ax.legend()
        self.fig.canvas.draw()

class KinematicsControlGUI(QMainWindow):
    """運動學控制主介面"""
    
    def __init__(self):
        super().__init__()
        self.dh_params = DHParameters()
        self.forward_kin = ForwardKinematics(self.dh_params)
        self.inverse_kin = InverseKinematics(self.dh_params, self.forward_kin)
        self.path_planner = PathPlanner(self.forward_kin, self.inverse_kin)
        
        # current_angles 表示伺服馬達的絕對角度 (0-180度)
        self.current_angles = np.array([90, 90, 90, 90])  # 伺服馬達初始位置
        self.waypoints = []
        self.trajectory_points = []
        self.trajectory_angles = []
        
        self.init_ui()
        self.update_display()
        
    def init_ui(self):
        """初始化UI"""
        self.setWindowTitle("機械手臂運動學控制系統 v1.0")
        self.setGeometry(100, 100, 1400, 800)
        
        # 主要widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左側控制面板
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # 右側3D視覺化
        right_panel = self.create_visualization_panel()
        main_layout.addWidget(right_panel, 2)
        
    def create_control_panel(self):
        """創建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 標籤頁
        tabs = QTabWidget()
        
        # 關節控制標籤頁
        joint_tab = self.create_joint_control_tab()
        tabs.addTab(joint_tab, "關節控制")
        
        # 點位記錄標籤頁
        waypoint_tab = self.create_waypoint_tab()
        tabs.addTab(waypoint_tab, "點位記錄")
        
        # DH參數標籤頁
        dh_tab = self.create_dh_parameters_tab()
        tabs.addTab(dh_tab, "DH參數")
        
        # 路徑規劃標籤頁
        path_tab = self.create_path_planning_tab()
        tabs.addTab(path_tab, "路徑規劃")
        
        layout.addWidget(tabs)
        return panel
    
    def create_joint_control_tab(self):
        """創建關節控制標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 關節角度控制
        joint_group = QGroupBox("關節角度控制")
        joint_layout = QGridLayout(joint_group)
        
        self.joint_sliders = []
        self.joint_spinboxes = []
        
        joint_names = ["關節1(底座)", "關節2(大臂)", "關節3(小臂)", "關節4(夾爪)"]
        
        for i, name in enumerate(joint_names):
            # 標籤
            joint_layout.addWidget(QLabel(name), i, 0)
            
            # 滑動條
            slider = QSlider(Qt.Horizontal)
            # 滑動條顯示伺服馬達角度 (20-160度)
            slider.setMinimum(20)
            slider.setMaximum(160)
            slider.setValue(90)  # 伺服馬達初始位置
            slider.valueChanged.connect(lambda v, idx=i: self.on_joint_changed(idx, v))
            self.joint_sliders.append(slider)
            joint_layout.addWidget(slider, i, 1)
            
            # 數值框
            spinbox = QSpinBox()
            # 數值框顯示伺服馬達角度 (20-160度)
            spinbox.setMinimum(20)
            spinbox.setMaximum(160)
            spinbox.setValue(90)  # 伺服馬達初始位置
            spinbox.setSuffix("°")
            spinbox.valueChanged.connect(lambda v, idx=i: self.on_joint_spinbox_changed(idx, v))
            self.joint_spinboxes.append(spinbox)
            joint_layout.addWidget(spinbox, i, 2)
        
        layout.addWidget(joint_group)
        
        # 末端效應器資訊
        ee_group = QGroupBox("末端效應器位置")
        ee_layout = QGridLayout(ee_group)
        
        ee_layout.addWidget(QLabel("X:"), 0, 0)
        self.ee_x_label = QLabel("0.0 mm")
        ee_layout.addWidget(self.ee_x_label, 0, 1)
        
        ee_layout.addWidget(QLabel("Y:"), 1, 0)
        self.ee_y_label = QLabel("0.0 mm")
        ee_layout.addWidget(self.ee_y_label, 1, 1)
        
        ee_layout.addWidget(QLabel("Z:"), 2, 0)
        self.ee_z_label = QLabel("0.0 mm")
        ee_layout.addWidget(self.ee_z_label, 2, 1)
        
        layout.addWidget(ee_group)
        
        return tab
    
    def create_waypoint_tab(self):
        """創建點位記錄標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 當前位置記錄
        current_group = QGroupBox("當前位置記錄")
        current_layout = QGridLayout(current_group)
        
        btn_record = QPushButton("記錄當前點位")
        btn_record.clicked.connect(self.record_current_waypoint)
        current_layout.addWidget(btn_record, 0, 0, 1, 2)
        
        layout.addWidget(current_group)
        
        # 手動輸入點位
        manual_group = QGroupBox("手動輸入點位")
        manual_layout = QGridLayout(manual_group)
        
        manual_layout.addWidget(QLabel("X:"), 0, 0)
        self.manual_x = QDoubleSpinBox()
        self.manual_x.setRange(-500, 500)
        self.manual_x.setSuffix(" mm")
        manual_layout.addWidget(self.manual_x, 0, 1)
        
        manual_layout.addWidget(QLabel("Y:"), 1, 0)
        self.manual_y = QDoubleSpinBox()
        self.manual_y.setRange(-500, 500)
        self.manual_y.setSuffix(" mm")
        manual_layout.addWidget(self.manual_y, 1, 1)
        
        manual_layout.addWidget(QLabel("Z:"), 2, 0)
        self.manual_z = QDoubleSpinBox()
        self.manual_z.setRange(0, 500)
        self.manual_z.setSuffix(" mm")
        manual_layout.addWidget(self.manual_z, 2, 1)
        
        btn_add_manual = QPushButton("新增點位")
        btn_add_manual.clicked.connect(self.add_manual_waypoint)
        manual_layout.addWidget(btn_add_manual, 3, 0, 1, 2)
        
        layout.addWidget(manual_group)
        
        # 點位清單
        waypoint_group = QGroupBox("點位清單")
        waypoint_layout = QVBoxLayout(waypoint_group)
        
        self.waypoint_table = QTableWidget(0, 4)
        self.waypoint_table.setHorizontalHeaderLabels(["#", "X (mm)", "Y (mm)", "Z (mm)"])
        waypoint_layout.addWidget(self.waypoint_table)
        
        # 清單控制按鈕
        btn_layout = QHBoxLayout()
        
        btn_clear = QPushButton("清除全部")
        btn_clear.clicked.connect(self.clear_waypoints)
        btn_layout.addWidget(btn_clear)
        
        btn_delete = QPushButton("刪除選中")
        btn_delete.clicked.connect(self.delete_selected_waypoint)
        btn_layout.addWidget(btn_delete)
        
        btn_save = QPushButton("儲存點位")
        btn_save.clicked.connect(self.save_waypoints)
        btn_layout.addWidget(btn_save)
        
        btn_load = QPushButton("載入點位")
        btn_load.clicked.connect(self.load_waypoints)
        btn_layout.addWidget(btn_load)
        
        waypoint_layout.addLayout(btn_layout)
        layout.addWidget(waypoint_group)
        
        return tab
    
    def create_dh_parameters_tab(self):
        """創建DH參數標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        dh_group = QGroupBox("DH參數表 (可編輯)")
        dh_layout = QVBoxLayout(dh_group)
        
        self.dh_table = QTableWidget(5, 4)
        self.dh_table.setHorizontalHeaderLabels(["θ (deg)", "d (mm)", "a (mm)", "α (deg)"])
        self.dh_table.setVerticalHeaderLabels(["關節1", "關節2", "關節3", "關節4", "末端校正"])
        
        # 填入當前DH參數
        for i in range(5):
            for j in range(4):
                item = QTableWidgetItem(str(self.dh_params.dh_table[i, j]))
                self.dh_table.setItem(i, j, item)
        
        dh_layout.addWidget(self.dh_table)
        
        # 控制按鈕
        btn_layout = QHBoxLayout()
        
        btn_update = QPushButton("更新參數")
        btn_update.clicked.connect(self.update_dh_parameters)
        btn_layout.addWidget(btn_update)
        
        btn_reset = QPushButton("重置預設")
        btn_reset.clicked.connect(self.reset_dh_parameters)
        btn_layout.addWidget(btn_reset)
        
        dh_layout.addLayout(btn_layout)
        layout.addWidget(dh_group)
        
        return tab
    
    def create_path_planning_tab(self):
        """創建路徑規劃標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 路徑規劃控制
        plan_group = QGroupBox("路徑規劃")
        plan_layout = QGridLayout(plan_group)
        
        btn_plan = QPushButton("規劃路徑")
        btn_plan.clicked.connect(self.plan_trajectory)
        plan_layout.addWidget(btn_plan, 0, 0)
        
        btn_simulate = QPushButton("動畫模擬")
        btn_simulate.clicked.connect(self.simulate_trajectory)
        plan_layout.addWidget(btn_simulate, 0, 1)
        
        btn_execute = QPushButton("執行路徑")
        btn_execute.clicked.connect(self.execute_trajectory)
        plan_layout.addWidget(btn_execute, 1, 0)
        
        self.auto_execute_cb = QCheckBox("自動執行")
        plan_layout.addWidget(self.auto_execute_cb, 1, 1)
        
        layout.addWidget(plan_group)
        
        # 軌跡資訊
        info_group = QGroupBox("軌跡資訊")
        info_layout = QVBoxLayout(info_group)
        
        self.trajectory_info = QTextEdit()
        self.trajectory_info.setMaximumHeight(200)
        info_layout.addWidget(self.trajectory_info)
        
        layout.addWidget(info_group)
        
        return tab
    
    def create_visualization_panel(self):
        """創建3D視覺化面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 建立matplotlib畫布
        self.robot_visualizer = RobotVisualizer(self.forward_kin)
        canvas = FigureCanvas(self.robot_visualizer.fig)
        layout.addWidget(canvas)
        
        return panel
    
    def on_joint_changed(self, joint_index, value):
        """關節角度改變時的處理"""
        # 直接使用絕對角度值
        self.current_angles[joint_index] = value
        self.joint_spinboxes[joint_index].setValue(value)
        self.update_display()
    
    def on_joint_spinbox_changed(self, joint_index, value):
        """關節數值框改變時的處理"""
        # 直接使用絕對角度值
        self.current_angles[joint_index] = value
        self.joint_sliders[joint_index].setValue(value)
        self.update_display()
    
    def update_display(self):
        """更新顯示"""
        # 計算正向運動學
        end_pos, _, _ = self.forward_kin.forward_kinematics(self.current_angles)
        
        # 更新末端效應器位置顯示
        self.ee_x_label.setText(f"{end_pos[0]:.1f} mm")
        self.ee_y_label.setText(f"{end_pos[1]:.1f} mm") 
        self.ee_z_label.setText(f"{end_pos[2]:.1f} mm")
        
        # 更新3D視覺化
        self.robot_visualizer.draw_robot(self.current_angles, self.waypoints, self.trajectory_points)
    
    def record_current_waypoint(self):
        """記錄當前點位"""
        end_pos, _, _ = self.forward_kin.forward_kinematics(self.current_angles)
        self.waypoints.append(end_pos.copy())
        self.update_waypoint_table()
        self.update_display()
    
    def add_manual_waypoint(self):
        """新增手動輸入的點位"""
        x = self.manual_x.value()
        y = self.manual_y.value()
        z = self.manual_z.value()
        self.waypoints.append(np.array([x, y, z]))
        self.update_waypoint_table()
        self.update_display()
    
    def update_waypoint_table(self):
        """更新點位表格"""
        self.waypoint_table.setRowCount(len(self.waypoints))
        for i, wp in enumerate(self.waypoints):
            self.waypoint_table.setItem(i, 0, QTableWidgetItem(str(i+1)))
            self.waypoint_table.setItem(i, 1, QTableWidgetItem(f"{wp[0]:.1f}"))
            self.waypoint_table.setItem(i, 2, QTableWidgetItem(f"{wp[1]:.1f}"))
            self.waypoint_table.setItem(i, 3, QTableWidgetItem(f"{wp[2]:.1f}"))
    
    def clear_waypoints(self):
        """清除所有點位"""
        self.waypoints.clear()
        self.trajectory_points.clear()
        self.trajectory_angles.clear()
        self.update_waypoint_table()
        self.update_display()
    
    def delete_selected_waypoint(self):
        """刪除選中的點位"""
        current_row = self.waypoint_table.currentRow()
        if current_row >= 0 and current_row < len(self.waypoints):
            del self.waypoints[current_row]
            self.update_waypoint_table()
            self.update_display()
    
    def save_waypoints(self):
        """儲存點位到檔案"""
        filename, _ = QFileDialog.getSaveFileName(self, "儲存點位", "", "JSON Files (*.json)")
        if filename:
            data = {
                'waypoints': [wp.tolist() for wp in self.waypoints],
                'dh_parameters': self.dh_params.dh_table.tolist()
            }
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
    
    def load_waypoints(self):
        """從檔案載入點位"""
        filename, _ = QFileDialog.getOpenFileName(self, "載入點位", "", "JSON Files (*.json)")
        if filename:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.waypoints = [np.array(wp) for wp in data['waypoints']]
            self.update_waypoint_table()
            self.update_display()
    
    def update_dh_parameters(self):
        """更新DH參數"""
        for i in range(5):
            for j in range(4):
                item = self.dh_table.item(i, j)
                if item:
                    self.dh_params.dh_table[i, j] = float(item.text())
    
    def reset_dh_parameters(self):
        """重置DH參數為預設值"""
        self.dh_params = DHParameters()
        for i in range(5):
            for j in range(4):
                item = QTableWidgetItem(str(self.dh_params.dh_table[i, j]))
                self.dh_table.setItem(i, j, item)
    
    def plan_trajectory(self):
        """規劃軌跡"""
        if len(self.waypoints) < 2:
            QMessageBox.warning(self, "警告", "至少需要2個點位才能規劃軌跡")
            return
        
        # 規劃軌跡
        self.trajectory_points, self.trajectory_angles = self.path_planner.generate_trajectory(
            self.waypoints, self.current_angles
        )
        
        # 更新軌跡資訊
        info_text = f"軌跡規劃完成！\n"
        info_text += f"點位數量: {len(self.waypoints)}\n"
        info_text += f"軌跡點數: {len(self.trajectory_points)}\n\n"
        
        for i, (point, angles) in enumerate(zip(self.trajectory_points, self.trajectory_angles)):
            info_text += f"點{i+1}: X={point[0]:.1f}, Y={point[1]:.1f}, Z={point[2]:.1f}\n"
            info_text += f"  角度: [{angles[0]:.1f}°, {angles[1]:.1f}°, {angles[2]:.1f}°, {angles[3]:.1f}°]\n"
        
        self.trajectory_info.setText(info_text)
        self.update_display()
    
    def simulate_trajectory(self):
        """動畫模擬軌跡"""
        if len(self.trajectory_angles) == 0:
            QMessageBox.warning(self, "警告", "請先規劃軌跡")
            return
        
        # 創建動畫視窗
        self.create_animation_window()
    
    def create_animation_window(self):
        """創建動畫視窗"""
        self.anim_fig, self.anim_ax = plt.subplots(figsize=(10, 8), subplot_kw={'projection': '3d'})
        
        def animate(frame):
            if frame < len(self.trajectory_angles):
                angles = self.trajectory_angles[frame]
                self.anim_ax.clear()
                
                # 設置座標軸
                limit = 400
                self.anim_ax.set_xlim([-limit, limit])
                self.anim_ax.set_ylim([-limit, limit])
                self.anim_ax.set_zlim([0, limit])
                self.anim_ax.set_xlabel('X (mm)')
                self.anim_ax.set_ylabel('Y (mm)')
                self.anim_ax.set_zlabel('Z (mm)')
                self.anim_ax.set_title(f'機械手臂軌跡模擬 - 第{frame+1}/{len(self.trajectory_angles)}步')
                
                # 繪製機械手臂
                positions = self.forward_kin.get_all_joint_positions(angles)
                self.anim_ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                                'b-o', linewidth=3, markersize=8)
                
                # 繪製軌跡
                if frame > 0:
                    past_points = self.trajectory_points[:frame+1]
                    past_points = np.array(past_points)
                    self.anim_ax.plot(past_points[:, 0], past_points[:, 1], past_points[:, 2], 
                                    'g-', linewidth=2, alpha=0.7)
                
                # 繪製目標點位
                if len(self.waypoints) > 0:
                    waypoints = np.array(self.waypoints)
                    self.anim_ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], 
                                       s=100, c='red', marker='^')
        
        # 創建動畫並保存引用以防止垃圾回收
        self.animation = animation.FuncAnimation(self.anim_fig, animate, frames=len(self.trajectory_angles), 
                                     interval=500, repeat=True)
        plt.show()
    
    def execute_trajectory(self):
        """執行軌跡 (發送到實際機械手臂)"""
        if len(self.trajectory_angles) == 0:
            QMessageBox.warning(self, "警告", "請先規劃軌跡")
            return
        
        reply = QMessageBox.question(self, "確認執行", 
                                   f"即將執行包含{len(self.trajectory_angles)}個動作的軌跡\n"
                                   "確定要控制實際機械手臂嗎？",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 這裡可以連接到實際的串列埠控制
            # 暫時只是顯示角度序列
            print("執行軌跡:")
            for i, angles in enumerate(self.trajectory_angles):
                print(f"步驟{i+1}: {angles}")
            
            QMessageBox.information(self, "執行完成", "軌跡執行完畢")

def main():
    app = QApplication(sys.argv)
    
    # 設置應用程式樣式
    app.setStyle('Fusion')
    
    # 創建主視窗
    window = KinematicsControlGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()