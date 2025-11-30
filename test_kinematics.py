#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
測試運動學計算
"""

import numpy as np
from kinematics_control import DHParameters, ForwardKinematics, InverseKinematics

def test_forward_kinematics():
    """測試正向運動學"""
    print("=== 正向運動學測試 ===")
    
    # 初始化
    dh_params = DHParameters()
    forward_kin = ForwardKinematics(dh_params)
    
    # 測試初始位置 (伺服馬達都在90度)
    servo_angles = [90, 90, 90, 90]
    print(f"伺服馬達角度: {servo_angles}")
    
    # 顯示DH參數
    dh_current = dh_params.get_dh_params(servo_angles)
    print(f"DH參數表:")
    print("關節  theta    d     a    alpha")
    for i, row in enumerate(dh_current):
        print(f"  {i+1}   {row[0]:6.1f} {row[1]:4.0f} {row[2]:4.0f} {row[3]:6.1f}")
    
    # 計算末端位置
    end_pos, _, _ = forward_kin.forward_kinematics(servo_angles)
    print(f"\n末端效應器位置: [{end_pos[0]:.1f}, {end_pos[1]:.1f}, {end_pos[2]:.1f}]")
    
    return end_pos

def test_inverse_kinematics():
    """測試逆向運動學"""
    print("\n=== 逆向運動學測試 ===")
    
    # 初始化
    dh_params = DHParameters()
    forward_kin = ForwardKinematics(dh_params)
    inverse_kin = InverseKinematics(dh_params, forward_kin)
    
    # 測試目標位置 (從JSON文件中的第一個點)
    target_pos = np.array([195.0, 0.0, 70.0])
    print(f"目標位置: {target_pos}")
    
    # 執行逆向運動學
    result_angles, success, iterations = inverse_kin.inverse_kinematics_iterative(target_pos)
    
    print(f"結果: 成功={success}, 迭代次數={iterations}")
    print(f"計算得到的角度: {result_angles}")
    
    # 驗證結果
    if success:
        verify_pos, _, _ = forward_kin.forward_kinematics(result_angles)
        error = np.linalg.norm(verify_pos - target_pos)
        print(f"驗證位置: {verify_pos}")
        print(f"位置誤差: {error:.6f}")

def test_workspace():
    """測試工作空間"""
    print("\n=== 工作空間測試 ===")
    
    dh_params = DHParameters()
    forward_kin = ForwardKinematics(dh_params)
    
    # 測試幾個不同的角度組合
    test_angles = [
        [90, 90, 90, 90],  # 初始位置
        [90, 60, 90, 90],  # 大臂向上
        [90, 120, 90, 90], # 大臂向下
        [45, 90, 90, 90],  # 底座旋轉
        [135, 90, 90, 90], # 底座反向旋轉
    ]
    
    for i, angles in enumerate(test_angles):
        pos, _, _ = forward_kin.forward_kinematics(angles)
        reach = np.linalg.norm(pos[:2])  # 水平距離
        print(f"角度組合{i+1} {angles} -> 位置 [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}], 水平距離: {reach:.1f}")

if __name__ == "__main__":
    test_forward_kinematics()
    test_inverse_kinematics() 
    test_workspace()