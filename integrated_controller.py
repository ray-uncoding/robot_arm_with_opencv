#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
機械手臂整合控制系統
結合運動學控制與實際串列埠通訊
"""

import sys
import time
import numpy as np
import serial
import serial.tools.list_ports
import cv2
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLabel, QPushButton, QGroupBox, QTextEdit, 
                             QTabWidget, QMessageBox, QComboBox, QCheckBox, QGridLayout,
                             QSlider, QSpinBox)
from PyQt5.QtCore import Qt, pyqtSignal, QThread, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap

# 導入運動學控制系統
from kinematics_control import (KinematicsControlGUI, DHParameters, 
                               ForwardKinematics, InverseKinematics, PathPlanner)

class IntegratedRobotController(QMainWindow):
    """整合機械手臂控制器"""
    
    def __init__(self):
        super().__init__()
        
        # 初始化運動學系統
        self.dh_params = DHParameters()
        self.forward_kin = ForwardKinematics(self.dh_params)
        self.inverse_kin = InverseKinematics(self.dh_params, self.forward_kin)
        self.path_planner = PathPlanner(self.forward_kin, self.inverse_kin)
        
        # 串列埠連接
        self.serial_port = None
        self.is_connected = False
        
        # 軌跡執行相關
        self.trajectory_angles = []
        self.execution_timer = QTimer()
        self.execution_timer.timeout.connect(self.execute_next_step)
        self.current_step = 0
        
        # 視覺監控相關
        self.camera = None
        self.camera_timer = QTimer()
        self.camera_timer.timeout.connect(self.update_camera_frame)
        self.is_camera_running = False
        
        # 實時控制相關 (伺服馬達角度)
        self.real_time_control = False
        self.last_angles = [90, 90, 90, 90]  # 伺服馬達初始位置
        self.send_delay_timer = QTimer()
        self.send_delay_timer.setSingleShot(True)
        self.send_delay_timer.timeout.connect(self.delayed_send_angles)
        self.pending_angles = None
        
        self.init_ui()
        
    def init_ui(self):
        """初始化UI"""
        self.setWindowTitle("機械手臂整合控制系統 v1.0")
        self.setGeometry(100, 100, 1200, 800)
        
        # 主要widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)
        
        # 連接狀態面板
        connection_panel = self.create_connection_panel()
        layout.addWidget(connection_panel)
        
        # 標籤頁
        tabs = QTabWidget()
        
        # 運動學控制標籤頁
        kinematics_tab = self.create_kinematics_tab()
        tabs.addTab(kinematics_tab, "運動學控制")
        
        # 軌跡執行標籤頁
        execution_tab = self.create_execution_tab()
        tabs.addTab(execution_tab, "軌跡執行")
        
        # 視覺監控標籤頁
        vision_tab = self.create_vision_tab()
        tabs.addTab(vision_tab, "視覺監控")
        
        layout.addWidget(tabs)
        
    def create_connection_panel(self):
        """創建連接面板"""
        panel = QGroupBox("Arduino連接")
        layout = QHBoxLayout(panel)
        
        # 串列埠選擇
        self.port_combo = QComboBox()
        self.refresh_ports()
        layout.addWidget(QLabel("串列埠:"))
        layout.addWidget(self.port_combo)
        
        # 連接按鈕
        self.btn_refresh = QPushButton("重新整理")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(self.btn_refresh)
        
        self.btn_connect = QPushButton("連接")
        self.btn_connect.clicked.connect(self.connect_arduino)
        layout.addWidget(self.btn_connect)
        
        self.btn_disconnect = QPushButton("斷開")
        self.btn_disconnect.clicked.connect(self.disconnect_arduino)
        self.btn_disconnect.setEnabled(False)
        layout.addWidget(self.btn_disconnect)
        
        # 狀態指示
        self.connection_status = QLabel("未連接")
        self.connection_status.setStyleSheet("QLabel { background-color: red; color: white; padding: 5px; border-radius: 3px; }")
        layout.addWidget(self.connection_status)
        
        # 實時控制開關
        from PyQt5.QtWidgets import QCheckBox
        self.realtime_checkbox = QCheckBox("實時控制")
        self.realtime_checkbox.setToolTip("開啟後，拖拉滑動條時實體機械手臂會同步動作")
        self.realtime_checkbox.toggled.connect(self.toggle_realtime_control)
        layout.addWidget(self.realtime_checkbox)
        
        return panel
        
    def create_kinematics_tab(self):
        """創建運動學控制標籤頁"""
        # 直接嵌入運動學控制GUI
        kinematics_widget = KinematicsControlGUI()
        
        # 儲存運動學控制widget的參考
        self.kinematics_widget = kinematics_widget
        
        # 覆寫規劃軌跡函數以同步數據和啟用執行按鈕
        original_plan = kinematics_widget.plan_trajectory
        
        def integrated_plan():
            # 執行原始規劃
            original_plan()
            
            # 同步軌跡數據到主控制器
            if hasattr(kinematics_widget, 'trajectory_angles') and len(kinematics_widget.trajectory_angles) > 0:
                self.trajectory_angles = []
                for angles in kinematics_widget.trajectory_angles:
                    # 確保只取前4個關節角度
                    if len(angles) >= 4:
                        self.trajectory_angles.append(angles[:4])
                    else:
                        self.trajectory_angles.append(angles)
                
                # 啟用執行按鈕
                if hasattr(self, 'btn_start_execution'):
                    self.btn_start_execution.setEnabled(True)
                
                # 記錄到日誌
                self.log_message(f"軌跡規劃完成，共{len(self.trajectory_angles)}步，已同步到執行標籤頁")
                self.log_message("請切換到「軌跡執行」標籤頁開始執行")
        
        kinematics_widget.plan_trajectory = integrated_plan
        
        # 修改執行軌跡函數以使用實際串列埠
        original_execute = kinematics_widget.execute_trajectory
        
        def integrated_execute():
            # 檢查連接狀態
            if not self.is_connected:
                QMessageBox.warning(self, "連接錯誤", "請先連接Arduino")
                return
                
            # 獲取軌跡角度並確保只使用前4個關節（可控關節）
            if hasattr(kinematics_widget, 'trajectory_angles') and len(kinematics_widget.trajectory_angles) > 0:
                self.trajectory_angles = []
                for angles in kinematics_widget.trajectory_angles:
                    # 確保只取前4個關節角度
                    if len(angles) >= 4:
                        self.trajectory_angles.append(angles[:4])
                    else:
                        self.trajectory_angles.append(angles)
                
                # 顯示軌跡預覽
                self.log_message(f"準備執行軌跡，共{len(self.trajectory_angles)}步")
                for i, angles in enumerate(self.trajectory_angles):
                    angle_str = ", ".join([f"{a:.1f}°" for a in angles])
                    self.log_message(f"  步驟{i+1}: [{angle_str}]")
            else:
                QMessageBox.warning(self, "軌跡錯誤", "請先規劃軌跡")
                return
                
            if len(self.trajectory_angles) == 0:
                QMessageBox.warning(self, "軌跡錯誤", "沒有可執行的軌跡")
                return
            
            # 確認執行
            reply = QMessageBox.question(self, "確認執行", 
                                       f"即將執行包含{len(self.trajectory_angles)}步的軌跡\n"
                                       f"每步間隔0.5秒，預計{len(self.trajectory_angles)*0.5:.1f}秒完成\n"
                                       "確定要控制實際機械手臂嗎？",
                                       QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                self.start_trajectory_execution()
            else:
                self.log_message("用戶取消執行軌跡")
        
        kinematics_widget.execute_trajectory = integrated_execute
        
        # 連接關節角度改變信號以實現實時控制
        self.connect_joint_signals(kinematics_widget)
        
        return kinematics_widget
    
    def create_execution_tab(self):
        """創建軌跡執行標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 執行控制
        control_group = QGroupBox("軌跡執行控制")
        control_layout = QGridLayout(control_group)
        
        # 第一行：載入和執行
        self.btn_load_trajectory = QPushButton("從運動學載入軌跡")
        self.btn_load_trajectory.clicked.connect(self.load_trajectory_from_kinematics)
        control_layout.addWidget(self.btn_load_trajectory, 0, 0)
        
        self.btn_start_execution = QPushButton("開始執行軌跡")
        self.btn_start_execution.clicked.connect(self.start_trajectory_execution)
        self.btn_start_execution.setEnabled(False)
        control_layout.addWidget(self.btn_start_execution, 0, 1)
        
        # 第二行：停止和測試
        self.btn_stop_execution = QPushButton("停止執行")
        self.btn_stop_execution.clicked.connect(self.stop_trajectory_execution)
        self.btn_stop_execution.setEnabled(False)
        control_layout.addWidget(self.btn_stop_execution, 1, 0)
        
        self.btn_emergency_stop = QPushButton("緊急停止")
        self.btn_emergency_stop.clicked.connect(self.emergency_stop)
        self.btn_emergency_stop.setStyleSheet("QPushButton { background-color: red; color: white; font-weight: bold; }")
        control_layout.addWidget(self.btn_emergency_stop, 1, 1)
        
        # 第三行：測試按鈕
        self.btn_manual_test = QPushButton("測試發送當前角度")
        self.btn_manual_test.clicked.connect(self.manual_send_test)
        self.btn_manual_test.setStyleSheet("QPushButton { background-color: orange; color: white; }")
        control_layout.addWidget(self.btn_manual_test, 2, 0, 1, 2)
        
        layout.addWidget(control_group)
        
        # 執行狀態
        status_group = QGroupBox("執行狀態")
        status_layout = QVBoxLayout(status_group)
        
        self.execution_info = QLabel("等待軌跡...")
        self.execution_info.setAlignment(Qt.AlignCenter)
        self.execution_info.setFont(QFont("Arial", 12))
        status_layout.addWidget(self.execution_info)
        
        self.progress_info = QLabel("0 / 0")
        self.progress_info.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.progress_info)
        
        layout.addWidget(status_group)
        
        # 通訊記錄
        log_group = QGroupBox("通訊記錄")
        log_layout = QVBoxLayout(log_group)
        
        self.communication_log = QTextEdit()
        self.communication_log.setMaximumHeight(300)
        log_layout.addWidget(self.communication_log)
        
        btn_clear_log = QPushButton("清除記錄")
        btn_clear_log.clicked.connect(self.communication_log.clear)
        log_layout.addWidget(btn_clear_log)
        
        layout.addWidget(log_group)
        
        return tab
    
    def manual_send_test(self):
        """手動測試發送角度"""
        if self.is_connected and hasattr(self, 'kinematics_widget'):
            if hasattr(self.kinematics_widget, 'current_angles'):
                angles = self.kinematics_widget.current_angles[:4]
                self.safe_log_message(f"手動測試發送: {angles}")
                self.send_angles_to_arduino(angles, "手動測試")
            else:
                self.safe_log_message("無法獲取當前角度")
        else:
            self.safe_log_message("請先連接Arduino")
    
    def load_trajectory_from_kinematics(self):
        """從運動學控制器載入軌跡"""
        if hasattr(self, 'kinematics_widget'):
            if hasattr(self.kinematics_widget, 'trajectory_angles') and len(self.kinematics_widget.trajectory_angles) > 0:
                self.trajectory_angles = []
                for angles in self.kinematics_widget.trajectory_angles:
                    # 確保只取前4個關節角度
                    if len(angles) >= 4:
                        self.trajectory_angles.append(angles[:4])
                    else:
                        self.trajectory_angles.append(angles)
                
                # 啟用執行按鈕
                self.btn_start_execution.setEnabled(True)
                
                # 更新顯示
                self.execution_info.setText(f"已載入軌跡\n共{len(self.trajectory_angles)}步")
                self.progress_info.setText(f"0 / {len(self.trajectory_angles)}")
                
                # 顯示軌跡預覽
                self.log_message(f"=== 已載入軌跡，共{len(self.trajectory_angles)}步 ===")
                for i, angles in enumerate(self.trajectory_angles):
                    angle_str = ", ".join([f"{a:.1f}°" for a in angles])
                    self.log_message(f"步驟{i+1}: [{angle_str}]")
                
                self.log_message("軌跡載入完成，可以點擊「開始執行軌跡」")
            else:
                QMessageBox.warning(self, "載入失敗", "運動學控制器中沒有軌跡\n請先在「運動學控制」標籤頁規劃軌跡")
                self.log_message("載入失敗：沒有找到軌跡數據")
        else:
            QMessageBox.warning(self, "載入失敗", "運動學控制器未初始化")
            self.log_message("載入失敗：運動學控制器未初始化")
    
    def connect_joint_signals(self, kinematics_widget):
        """連接關節控制信號以實現實時控制"""
        try:
            # 儲存widget參考
            self.kinematics_widget = kinematics_widget
            
            # 直接覆寫更新顯示函數
            original_update_display = kinematics_widget.update_display
            
            def enhanced_update_display():
                # 執行原始更新函數
                original_update_display()
                
                # 如果啟用了實時控制，就發送新的角度
                if hasattr(self, 'real_time_control') and self.real_time_control and self.is_connected:
                    try:
                        # 獲取當前所有角度
                        if hasattr(kinematics_widget, 'current_angles'):
                            current_angles = kinematics_widget.current_angles.copy()
                            self.send_current_angles_with_delay(current_angles)
                    except Exception as e:
                        self.safe_log_message(f"實時控制發送失敗: {str(e)}")
            
            # 替換函數
            kinematics_widget.update_display = enhanced_update_display
            
            self.safe_log_message("關節信號連接成功")
            
        except Exception as e:
            # 初始化時如果communication_log還沒有創建，就使用print
            if hasattr(self, 'communication_log'):
                self.log_message(f"連接關節信號失敗: {str(e)}")
            else:
                print(f"連接關節信號失敗: {str(e)}")
    
    def toggle_realtime_control(self, enabled):
        """切換實時控制模式"""
        self.real_time_control = enabled
        if enabled:
            if self.is_connected:
                self.safe_log_message("已啟用實時控制模式")
                # 立即發送當前角度（但要確保運動學widget已完全初始化）
                if hasattr(self, 'kinematics_widget') and hasattr(self.kinematics_widget, 'current_angles'):
                    self.safe_log_message(f"當前角度: {self.kinematics_widget.current_angles}")
                    self.send_current_angles_with_delay()
                else:
                    self.safe_log_message("等待運動學系統初始化完成...")
            else:
                if hasattr(self, 'realtime_checkbox'):
                    self.realtime_checkbox.setChecked(False)
                QMessageBox.warning(self, "實時控制", "請先連接Arduino再啟用實時控制")
        else:
            self.safe_log_message("已停用實時控制模式")
            self.send_delay_timer.stop()
    
    def on_joint_angle_changed(self, joint_index, angle):
        """關節角度改變時的處理（備用函數）"""
        # 這個函數現在主要用於connect_joint_signals中的替換方式
        if self.real_time_control and self.is_connected:
            self.safe_log_message(f"關節{joint_index}改變為{angle}°")
    
    def send_current_angles_with_delay(self, angles=None):
        """延遲發送當前角度（避免過於頻繁發送）"""
        if not self.real_time_control or not self.is_connected:
            return
        
        if angles is None:
            # 嘗試從運動學widget獲取當前角度
            if hasattr(self, 'kinematics_widget') and hasattr(self.kinematics_widget, 'current_angles'):
                angles = self.kinematics_widget.current_angles.copy()
            else:
                return  # 如果無法獲取角度，就不發送
        
        # 確保 angles 是一個有效的列表或數組
        if angles is not None:
            try:
                # 轉換為列表並只取前4個
                if hasattr(angles, 'tolist'):
                    angles = angles.tolist()
                if isinstance(angles, (list, tuple)) and len(angles) >= 4:
                    self.pending_angles = list(angles[:4])
                    # 重設定時器，避免過於頻繁發送
                    self.send_delay_timer.stop()
                    self.send_delay_timer.start(100)  # 100ms 延遲
            except Exception as e:
                self.safe_log_message(f"角度處理錯誤: {str(e)}")
    
    def delayed_send_angles(self):
        """延遲發送角度"""
        if (self.pending_angles is not None and 
            self.real_time_control and 
            self.is_connected):
            # 檢查角度是否有變化
            angles_changed = True
            if hasattr(self, 'last_angles') and self.last_angles is not None:
                try:
                    # 安全的數組比較
                    import numpy as np
                    angles_changed = not np.array_equal(self.pending_angles, self.last_angles)
                except:
                    # 如果比較失敗，就假設有變化
                    angles_changed = True
            
            if angles_changed:
                success = self.send_angles_to_arduino(self.pending_angles, log_prefix="即時")
                if success:
                    self.last_angles = self.pending_angles.copy() if self.pending_angles is not None else None
            
            self.pending_angles = None
    
    def refresh_ports(self):
        """重新整理串列埠"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def connect_arduino(self):
        """連接Arduino"""
        if self.port_combo.currentText():
            port = self.port_combo.currentText().split(' - ')[0]
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=1)
                time.sleep(2)  # 等待Arduino初始化
                self.is_connected = True
                
                self.connection_status.setText("已連接")
                self.connection_status.setStyleSheet("QLabel { background-color: green; color: white; padding: 5px; border-radius: 3px; }")
                
                self.btn_connect.setEnabled(False)
                self.btn_disconnect.setEnabled(True)
                self.btn_start_execution.setEnabled(True)
                self.realtime_checkbox.setEnabled(True)
                
                self.safe_log_message("Arduino連接成功")
                
            except Exception as e:
                QMessageBox.critical(self, "連接錯誤", f"無法連接Arduino:\n{str(e)}")
                self.safe_log_message(f"連接錯誤: {str(e)}")
    
    def disconnect_arduino(self):
        """斷開Arduino連接"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        
        self.is_connected = False
        self.connection_status.setText("未連接")
        self.connection_status.setStyleSheet("QLabel { background-color: red; color: white; padding: 5px; border-radius: 3px; }")
        
        self.btn_connect.setEnabled(True)
        self.btn_disconnect.setEnabled(False)
        self.btn_start_execution.setEnabled(False)
        self.realtime_checkbox.setEnabled(False)
        self.realtime_checkbox.setChecked(False)
        self.real_time_control = False
        self.send_delay_timer.stop()
        
        self.safe_log_message("Arduino連接已斷開")
    
    def send_angles_to_arduino(self, angles, log_prefix="發送"):
        """發送角度到Arduino"""
        if self.is_connected and self.serial_port:
            try:
                # 確保只使用前4個關節角度
                if len(angles) >= 4:
                    cmd_angles = angles[:4]
                else:
                    cmd_angles = list(angles) + [0] * (4 - len(angles))  # 補齊到4個
                
                command = f"{cmd_angles[0]:.1f},{cmd_angles[1]:.1f},{cmd_angles[2]:.1f},{cmd_angles[3]:.1f}\n"
                self.serial_port.write(command.encode())
                
                # 根據是否為實時控制調整日誌顯示
                if log_prefix == "即時":
                    # 實時控制時簡化日誌
                    angle_str = ", ".join([f"{a:.0f}°" for a in cmd_angles])
                    self.safe_log_message(f"即時: [{angle_str}]")
                else:
                    self.safe_log_message(f"{log_prefix}: {command.strip()}")
                
                # 讀取Arduino回應（實時控制時跳過以提高效率）
                if log_prefix != "即時":
                    time.sleep(0.1)
                    while self.serial_port.in_waiting:
                        response = self.serial_port.readline().decode().strip()
                        if response:
                            self.safe_log_message(f"Arduino: {response}")
                
                return True
            except Exception as e:
                self.safe_log_message(f"發送錯誤: {str(e)}")
                return False
        return False
    
    def start_trajectory_execution(self):
        """開始執行軌跡"""
        self.log_message("=== 開始執行軌跡 ===")
        self.log_message(f"連接狀態: {self.is_connected}")
        self.log_message(f"串口對象: {self.serial_port is not None}")
        self.log_message(f"軌跡步數: {len(self.trajectory_angles)}")
        
        if len(self.trajectory_angles) == 0:
            QMessageBox.warning(self, "軌跡錯誤", "沒有可執行的軌跡")
            return
        
        if not self.is_connected:
            QMessageBox.warning(self, "連接錯誤", "請先連接Arduino")
            return
        
        if not self.serial_port:
            QMessageBox.warning(self, "串口錯誤", "串口未初始化")
            return
        
        # 驗證軌跡數據
        valid_trajectory = True
        for i, angles in enumerate(self.trajectory_angles):
            if len(angles) < 4:
                self.log_message(f"警告: 軌跡點{i+1}角度不足4個，將補齊為0")
        
        self.current_step = 0
        self.btn_start_execution.setEnabled(False)
        self.btn_stop_execution.setEnabled(True)
        
        self.execution_info.setText("正在執行軌跡...")
        self.update_progress()
        
        # 開始定時執行 - 每個點停留0.5秒
        self.execution_timer.start(500)  # 500毫秒執行一步
        
        self.log_message(f"開始執行軌跡，共{len(self.trajectory_angles)}步")
        self.log_message(f"軌跡預覽: 每步間隔0.5秒，預計執行時間{len(self.trajectory_angles)*0.5:.1f}秒")
    
    def execute_next_step(self):
        """執行下一步"""
        if self.current_step < len(self.trajectory_angles):
            angles = self.trajectory_angles[self.current_step]
            
            # 顯示當前執行的角度信息
            angle_str = ", ".join([f"{a:.1f}°" for a in angles[:4]])
            self.execution_info.setText(f"執行第{self.current_step + 1}步\n角度: [{angle_str}]")
            self.log_message(f"步驟{self.current_step + 1}/{len(self.trajectory_angles)}: 發送角度 [{angle_str}]")
            
            success = self.send_angles_to_arduino(angles)
            
            if success:
                self.log_message(f"步驟{self.current_step + 1} 執行成功")
                self.current_step += 1
                self.update_progress()
            else:
                self.log_message(f"步驟{self.current_step + 1} 執行失敗")
                self.stop_trajectory_execution()
                QMessageBox.critical(self, "執行錯誤", "發送角度失敗，軌跡執行已停止")
        else:
            # 執行完成
            self.stop_trajectory_execution()
            self.execution_info.setText("軌跡執行完成！")
            QMessageBox.information(self, "執行完成", "軌跡已成功執行完畢")
            self.log_message("軌跡執行完成")
    
    def stop_trajectory_execution(self):
        """停止軌跡執行"""
        self.execution_timer.stop()
        self.btn_start_execution.setEnabled(True)
        self.btn_stop_execution.setEnabled(False)
        
        if self.current_step < len(self.trajectory_angles):
            self.execution_info.setText("軌跡執行已停止")
            self.log_message("軌跡執行已手動停止")
    
    def emergency_stop(self):
        """緊急停止"""
        self.stop_trajectory_execution()
        
        # 發送停止命令或回到安全位置
        if self.is_connected:
            safe_angles = [90, 90, 90, 90]  # 安全位置設為90度
            self.send_angles_to_arduino(safe_angles)
        
        self.execution_info.setText("緊急停止！")
        self.log_message("執行緊急停止")
        
        QMessageBox.warning(self, "緊急停止", "已執行緊急停止，機械手臂回到安全位置")
    
    def update_progress(self):
        """更新進度顯示"""
        total = len(self.trajectory_angles)
        current = self.current_step
        self.progress_info.setText(f"{current} / {total}")
    
    def log_message(self, message):
        """記錄訊息"""
        timestamp = time.strftime("%H:%M:%S")
        self.communication_log.append(f"[{timestamp}] {message}")
    
    def safe_log_message(self, message):
        """安全的日誌記錄，避免初始化時的錯誤"""
        if hasattr(self, 'communication_log'):
            self.log_message(message)
        else:
            print(f"[LOG] {message}")
    
    def closeEvent(self, event):
        """關閉程式時清理"""
        self.stop_trajectory_execution()
        self.stop_camera()  # 關閉攝影機
        if self.is_connected:
            self.disconnect_arduino()
        event.accept()
    
    def create_vision_tab(self):
        """創建視覺監控標籤頁"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 攝影機控制
        camera_control_group = QGroupBox("攝影機控制")
        camera_control_layout = QHBoxLayout(camera_control_group)
        
        self.btn_start_camera = QPushButton("開啟攝影機")
        self.btn_start_camera.clicked.connect(self.start_camera)
        camera_control_layout.addWidget(self.btn_start_camera)
        
        self.btn_stop_camera = QPushButton("關閉攝影機")
        self.btn_stop_camera.clicked.connect(self.stop_camera)
        self.btn_stop_camera.setEnabled(False)
        camera_control_layout.addWidget(self.btn_stop_camera)
        
        # 攝影機選擇
        camera_control_layout.addWidget(QLabel("攝影機ID:"))
        self.camera_id_spin = QSpinBox()
        self.camera_id_spin.setMinimum(0)
        self.camera_id_spin.setMaximum(5)
        self.camera_id_spin.setValue(0)
        camera_control_layout.addWidget(self.camera_id_spin)
        
        layout.addWidget(camera_control_group)
        
        # 紅點檢測參數
        detection_group = QGroupBox("紅點檢測參數")
        detection_layout = QGridLayout(detection_group)
        
        # HSV 下限
        detection_layout.addWidget(QLabel("H 下限:"), 0, 0)
        self.h_min_slider = QSlider(Qt.Horizontal)
        self.h_min_slider.setMinimum(0)
        self.h_min_slider.setMaximum(179)
        self.h_min_slider.setValue(0)
        detection_layout.addWidget(self.h_min_slider, 0, 1)
        self.h_min_label = QLabel("0")
        detection_layout.addWidget(self.h_min_label, 0, 2)
        self.h_min_slider.valueChanged.connect(lambda v: self.h_min_label.setText(str(v)))
        
        detection_layout.addWidget(QLabel("S 下限:"), 1, 0)
        self.s_min_slider = QSlider(Qt.Horizontal)
        self.s_min_slider.setMinimum(0)
        self.s_min_slider.setMaximum(255)
        self.s_min_slider.setValue(100)
        detection_layout.addWidget(self.s_min_slider, 1, 1)
        self.s_min_label = QLabel("100")
        detection_layout.addWidget(self.s_min_label, 1, 2)
        self.s_min_slider.valueChanged.connect(lambda v: self.s_min_label.setText(str(v)))
        
        detection_layout.addWidget(QLabel("V 下限:"), 2, 0)
        self.v_min_slider = QSlider(Qt.Horizontal)
        self.v_min_slider.setMinimum(0)
        self.v_min_slider.setMaximum(255)
        self.v_min_slider.setValue(100)
        detection_layout.addWidget(self.v_min_slider, 2, 1)
        self.v_min_label = QLabel("100")
        detection_layout.addWidget(self.v_min_label, 2, 2)
        self.v_min_slider.valueChanged.connect(lambda v: self.v_min_label.setText(str(v)))
        
        # HSV 上限
        detection_layout.addWidget(QLabel("H 上限:"), 0, 3)
        self.h_max_slider = QSlider(Qt.Horizontal)
        self.h_max_slider.setMinimum(0)
        self.h_max_slider.setMaximum(179)
        self.h_max_slider.setValue(10)
        detection_layout.addWidget(self.h_max_slider, 0, 4)
        self.h_max_label = QLabel("10")
        detection_layout.addWidget(self.h_max_label, 0, 5)
        self.h_max_slider.valueChanged.connect(lambda v: self.h_max_label.setText(str(v)))
        
        detection_layout.addWidget(QLabel("S 上限:"), 1, 3)
        self.s_max_slider = QSlider(Qt.Horizontal)
        self.s_max_slider.setMinimum(0)
        self.s_max_slider.setMaximum(255)
        self.s_max_slider.setValue(255)
        detection_layout.addWidget(self.s_max_slider, 1, 4)
        self.s_max_label = QLabel("255")
        detection_layout.addWidget(self.s_max_label, 1, 5)
        self.s_max_slider.valueChanged.connect(lambda v: self.s_max_label.setText(str(v)))
        
        detection_layout.addWidget(QLabel("V 上限:"), 2, 3)
        self.v_max_slider = QSlider(Qt.Horizontal)
        self.v_max_slider.setMinimum(0)
        self.v_max_slider.setMaximum(255)
        self.v_max_slider.setValue(255)
        detection_layout.addWidget(self.v_max_slider, 2, 4)
        self.v_max_label = QLabel("255")
        detection_layout.addWidget(self.v_max_label, 2, 5)
        self.v_max_slider.valueChanged.connect(lambda v: self.v_max_label.setText(str(v)))
        
        layout.addWidget(detection_group)
        
        # 視訊顯示
        video_group = QGroupBox("攝影機畫面")
        video_layout = QVBoxLayout(video_group)
        
        self.camera_label = QLabel("攝影機未啟動")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("QLabel { background-color: black; color: white; }")
        video_layout.addWidget(self.camera_label)
        
        # 檢測資訊
        self.detection_info = QLabel("檢測到的紅點數量: 0")
        self.detection_info.setAlignment(Qt.AlignCenter)
        video_layout.addWidget(self.detection_info)
        
        layout.addWidget(video_group)
        
        return tab
    
    def start_camera(self):
        """啟動攝影機"""
        camera_id = self.camera_id_spin.value()
        self.camera = cv2.VideoCapture(camera_id)
        
        if self.camera.isOpened():
            self.is_camera_running = True
            self.camera_timer.start(30)  # 30ms 更新一次 (約33 FPS)
            self.btn_start_camera.setEnabled(False)
            self.btn_stop_camera.setEnabled(True)
            self.log_message(f"攝影機 {camera_id} 已啟動")
        else:
            QMessageBox.warning(self, "錯誤", f"無法開啟攝影機 {camera_id}")
            self.log_message(f"無法開啟攝影機 {camera_id}")
    
    def stop_camera(self):
        """停止攝影機"""
        self.is_camera_running = False
        self.camera_timer.stop()
        
        if self.camera:
            self.camera.release()
            self.camera = None
        
        self.camera_label.setText("攝影機未啟動")
        self.btn_start_camera.setEnabled(True)
        self.btn_stop_camera.setEnabled(False)
        self.log_message("攝影機已關閉")
    
    def update_camera_frame(self):
        """更新攝影機畫面"""
        if not self.is_camera_running or not self.camera:
            return
        
        ret, frame = self.camera.read()
        if not ret:
            return
        
        # 轉換為HSV色彩空間
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 獲取HSV閾值
        lower_red = np.array([self.h_min_slider.value(), 
                             self.s_min_slider.value(), 
                             self.v_min_slider.value()])
        upper_red = np.array([self.h_max_slider.value(), 
                             self.s_max_slider.value(), 
                             self.v_max_slider.value()])
        
        # 創建紅色遮罩
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # 形態學操作去除雜訊
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 尋找輪廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 繪製檢測到的紅點
        red_dots = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # 過濾小面積
                # 計算中心點
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    red_dots.append((cx, cy))
                    
                    # 繪製圓圈和中心點
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    cv2.putText(frame, f"({cx},{cy})", (cx+15, cy), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 更新檢測資訊
        self.detection_info.setText(f"檢測到的紅點數量: {len(red_dots)}")
        
        # 轉換為Qt格式顯示
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        # 縮放以適應標籤大小
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
            self.camera_label.width(), 
            self.camera_label.height(), 
            Qt.KeepAspectRatio
        )
        self.camera_label.setPixmap(scaled_pixmap)

def main():
    app = QApplication(sys.argv)
    
    # 設置應用程式樣式
    app.setStyle('Fusion')
    
    # 設置中文字體以解決中文顯示問題
    try:
        import matplotlib
        matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
        matplotlib.rcParams['axes.unicode_minus'] = False
    except:
        pass
    
    # 創建主視窗
    window = IntegratedRobotController()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()