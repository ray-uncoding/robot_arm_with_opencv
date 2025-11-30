#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
機械手臂控制程式
使用 Qt5 圖形介面控制 Arduino 機械手臂
"""

import sys
import time
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QLabel, QSlider, QPushButton, QComboBox, 
                             QGroupBox, QTextEdit, QSpinBox, QGridLayout,
                             QMessageBox, QStatusBar, QFrame, QPlainTextEdit)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor

class SerialWorker(QThread):
    """Serial 通訊工作線程"""
    data_received = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
    def connect_serial(self, port, baudrate=115200):
        """連接串列埠"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # 等待 Arduino 初始化
            self.running = True
            self.connection_status.emit(True)
            return True
        except Exception as e:
            self.connection_status.emit(False)
            self.data_received.emit(f"連接錯誤: {str(e)}")
            return False
    
    def disconnect_serial(self):
        """斷開串列埠連接"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connection_status.emit(False)
    
    def send_command(self, command):
        """發送命令到 Arduino"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(command.encode())
                self.data_received.emit(f"發送: {command.strip()}")
                return True
            except Exception as e:
                self.data_received.emit(f"發送錯誤: {str(e)}")
                return False
        return False
    
    def run(self):
        """讀取串列埠數據"""
        while self.running and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.readline().decode().strip()
                    if data:
                        self.data_received.emit(f"Arduino: {data}")
            except Exception as e:
                self.data_received.emit(f"讀取錯誤: {str(e)}")
                break
            time.sleep(0.1)

class MotorControlWidget(QGroupBox):
    """單個馬達控制widget"""
    angle_changed = pyqtSignal(int, int)  # motor_index, angle
    
    def __init__(self, motor_index, motor_name):
        super().__init__(f"馬達 {motor_index}: {motor_name}")
        self.motor_index = motor_index
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        # 角度顯示
        self.angle_label = QLabel("90°")
        self.angle_label.setAlignment(Qt.AlignCenter)
        self.angle_label.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(self.angle_label)
        
        # 滑動條
        self.slider = QSlider(Qt.Vertical)
        self.slider.setMinimum(0)
        self.slider.setMaximum(180)
        self.slider.setValue(90)
        self.slider.setTickPosition(QSlider.TicksBothSides)
        self.slider.setTickInterval(30)
        self.slider.valueChanged.connect(self.on_slider_changed)
        layout.addWidget(self.slider)
        
        # 數值輸入
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("角度:"))
        self.spinbox = QSpinBox()
        self.spinbox.setMinimum(0)
        self.spinbox.setMaximum(180)
        self.spinbox.setValue(90)
        self.spinbox.setSuffix("°")
        self.spinbox.valueChanged.connect(self.on_spinbox_changed)
        input_layout.addWidget(self.spinbox)
        layout.addLayout(input_layout)
        
        # 預設位置按鈕
        button_layout = QHBoxLayout()
        self.btn_min = QPushButton("0°")
        self.btn_mid = QPushButton("90°")
        self.btn_max = QPushButton("180°")
        
        self.btn_min.clicked.connect(lambda: self.set_angle(0))
        self.btn_mid.clicked.connect(lambda: self.set_angle(90))
        self.btn_max.clicked.connect(lambda: self.set_angle(180))
        
        button_layout.addWidget(self.btn_min)
        button_layout.addWidget(self.btn_mid)
        button_layout.addWidget(self.btn_max)
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
    def on_slider_changed(self, value):
        """滑動條改變時"""
        self.angle_label.setText(f"{value}°")
        self.spinbox.setValue(value)
        self.angle_changed.emit(self.motor_index, value)
        
    def on_spinbox_changed(self, value):
        """數值框改變時"""
        self.angle_label.setText(f"{value}°")
        self.slider.setValue(value)
        self.angle_changed.emit(self.motor_index, value)
        
    def set_angle(self, angle):
        """設定角度"""
        self.slider.setValue(angle)
        
    def get_angle(self):
        """獲取當前角度"""
        return self.slider.value()

class RobotArmController(QMainWindow):
    """機械手臂控制主視窗"""
    
    def __init__(self):
        super().__init__()
        self.serial_worker = SerialWorker()
        self.motor_angles = [90, 90, 90, 90]  # 四個馬達的角度
        self.auto_send = True  # 是否自動發送
        
        self.init_ui()
        self.setup_connections()
        self.refresh_ports()
        
    def init_ui(self):
        """初始化UI"""
        self.setWindowTitle("機械手臂控制器 v1.0")
        self.setGeometry(100, 100, 800, 600)
        
        # 主要widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # 左側控制面板
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 2)
        
        # 右側狀態面板
        right_panel = self.create_status_panel()
        main_layout.addWidget(right_panel, 1)
        
        # 狀態列
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("準備就緒")
        
    def create_control_panel(self):
        """創建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 連接控制
        conn_group = QGroupBox("連接設定")
        conn_layout = QGridLayout(conn_group)
        
        conn_layout.addWidget(QLabel("串列埠:"), 0, 0)
        self.port_combo = QComboBox()
        conn_layout.addWidget(self.port_combo, 0, 1)
        
        self.btn_refresh = QPushButton("重新整理")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.btn_refresh, 0, 2)
        
        self.btn_connect = QPushButton("連接")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect, 1, 0)
        
        self.btn_disconnect = QPushButton("斷開")
        self.btn_disconnect.clicked.connect(self.disconnect)
        self.btn_disconnect.setEnabled(False)
        conn_layout.addWidget(self.btn_disconnect, 1, 1)
        
        layout.addWidget(conn_group)
        
        # 馬達控制
        motor_group = QGroupBox("馬達控制")
        motor_layout = QHBoxLayout(motor_group)
        
        motor_names = ["底座", "大臂", "小臂", "夾爪"]
        self.motor_controls = []
        
        for i, name in enumerate(motor_names):
            motor_control = MotorControlWidget(i, name)
            motor_control.angle_changed.connect(self.on_motor_angle_changed)
            self.motor_controls.append(motor_control)
            motor_layout.addWidget(motor_control)
            
        layout.addWidget(motor_group)
        
        # 控制按鈕
        control_group = QGroupBox("控制選項")
        control_layout = QGridLayout(control_group)
        
        self.btn_send_all = QPushButton("發送所有角度")
        self.btn_send_all.clicked.connect(self.send_all_angles)
        control_layout.addWidget(self.btn_send_all, 0, 0)
        
        self.btn_reset = QPushButton("重置為90°")
        self.btn_reset.clicked.connect(self.reset_all_motors)
        control_layout.addWidget(self.btn_reset, 0, 1)
        
        self.btn_home = QPushButton("回到原點")
        self.btn_home.clicked.connect(self.go_home)
        control_layout.addWidget(self.btn_home, 1, 0)
        
        self.btn_test = QPushButton("測試動作")
        self.btn_test.clicked.connect(self.test_movement)
        control_layout.addWidget(self.btn_test, 1, 1)
        
        layout.addWidget(control_group)
        
        return panel
        
    def create_status_panel(self):
        """創建狀態面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 連接狀態
        status_group = QGroupBox("連接狀態")
        status_layout = QVBoxLayout(status_group)
        
        self.status_label = QLabel("未連接")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("QLabel { background-color: red; color: white; padding: 10px; border-radius: 5px; }")
        status_layout.addWidget(self.status_label)
        
        layout.addWidget(status_group)
        
        # 通訊記錄
        log_group = QGroupBox("通訊記錄")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QPlainTextEdit()
        self.log_text.setMaximumBlockCount(1000)  # 限制行數
        self.log_text.setReadOnly(True)  # 設為只讀
        log_layout.addWidget(self.log_text)
        
        clear_btn = QPushButton("清除記錄")
        clear_btn.clicked.connect(self.log_text.clear)
        log_layout.addWidget(clear_btn)
        
        layout.addWidget(log_group)
        
        # 當前角度顯示
        angle_group = QGroupBox("當前角度")
        angle_layout = QVBoxLayout(angle_group)
        
        self.angle_display = QLabel("90°, 90°, 90°, 90°")
        self.angle_display.setAlignment(Qt.AlignCenter)
        self.angle_display.setFont(QFont("Arial", 12))
        angle_layout.addWidget(self.angle_display)
        
        layout.addWidget(angle_group)
        
        return panel
        
    def setup_connections(self):
        """設置信號連接"""
        self.serial_worker.data_received.connect(self.on_data_received)
        self.serial_worker.connection_status.connect(self.on_connection_status)
        
    def refresh_ports(self):
        """重新整理可用串列埠"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
            
    def toggle_connection(self):
        """切換連接狀態"""
        if self.port_combo.currentText():
            port = self.port_combo.currentText().split(' - ')[0]
            if self.serial_worker.connect_serial(port):
                self.serial_worker.start()
                
    def disconnect(self):
        """斷開連接"""
        self.serial_worker.disconnect_serial()
        self.serial_worker.quit()
        self.serial_worker.wait()
        
    def on_connection_status(self, connected):
        """連接狀態改變"""
        if connected:
            self.status_label.setText("已連接")
            self.status_label.setStyleSheet("QLabel { background-color: green; color: white; padding: 10px; border-radius: 5px; }")
            self.btn_connect.setEnabled(False)
            self.btn_disconnect.setEnabled(True)
            self.statusBar.showMessage("Arduino 已連接")
        else:
            self.status_label.setText("未連接")
            self.status_label.setStyleSheet("QLabel { background-color: red; color: white; padding: 10px; border-radius: 5px; }")
            self.btn_connect.setEnabled(True)
            self.btn_disconnect.setEnabled(False)
            self.statusBar.showMessage("Arduino 未連接")
            
    def on_data_received(self, data):
        """接收到數據"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.appendPlainText(f"[{timestamp}] {data}")
        
    def on_motor_angle_changed(self, motor_index, angle):
        """馬達角度改變"""
        self.motor_angles[motor_index] = angle
        self.update_angle_display()
        
        if self.auto_send:
            self.send_all_angles()
            
    def update_angle_display(self):
        """更新角度顯示"""
        angles_str = ", ".join([f"{angle}°" for angle in self.motor_angles])
        self.angle_display.setText(angles_str)
        
    def send_all_angles(self):
        """發送所有馬達角度"""
        command = ",".join([str(angle) for angle in self.motor_angles]) + "\n"
        self.serial_worker.send_command(command)
        
    def reset_all_motors(self):
        """重置所有馬達到90度"""
        for motor_control in self.motor_controls:
            motor_control.set_angle(90)
            
    def go_home(self):
        """回到原點位置"""
        home_angles = [90, 90, 90, 90]
        for i, motor_control in enumerate(self.motor_controls):
            motor_control.set_angle(home_angles[i])
            
    def test_movement(self):
        """測試動作序列"""
        test_sequence = [
            [90, 90, 90, 90],   # 原點
            [45, 45, 45, 45],   # 動作1
            [135, 135, 135, 135], # 動作2
            [90, 90, 90, 90],   # 回到原點
        ]
        
        reply = QMessageBox.question(self, "測試動作", 
                                   "是否執行測試動作序列？\n這將控制機械手臂移動。",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # 這裡可以加入動作序列的執行邏輯
            for angles in test_sequence:
                for i, angle in enumerate(angles):
                    self.motor_controls[i].set_angle(angle)
                time.sleep(1)  # 每個動作間隔1秒
                
    def closeEvent(self, event):
        """關閉程式時的清理工作"""
        if self.serial_worker.isRunning():
            self.disconnect()
        event.accept()

def main():
    app = QApplication(sys.argv)
    
    # 設置應用程式樣式
    app.setStyle('Fusion')
    
    # 創建主視窗
    window = RobotArmController()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()