#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
測試簡單的逆向運動學
"""

import numpy as np
from kinematics_control import DHParameters, ForwardKinematics, InverseKinematics

# 初始化
dh_params = DHParameters()
forward_kin = ForwardKinematics(dh_params)
inverse_kin = InverseKinematics(dh_params, forward_kin)

print("=== 測試1: 測試一個接近當前位置的點 ===")
target1 = np.array([180.0, 0.0, 80.0])
print(f"目標位置: {target1}")
angles1, success1, iter1 = inverse_kin.inverse_kinematics_iterative(target1, max_iterations=200)
print(f"結果: 成功={success1}, 迭代={iter1}, 角度={angles1}\n")

if success1:
    verify1, _, _ = forward_kin.forward_kinematics(angles1)
    print(f"驗證位置: {verify1}")
    print(f"誤差: {np.linalg.norm(verify1 - target1):.6f}\n")

print("=== 測試2: 測試底座旋轉45度的位置 ===")
# 先算出45度時的實際位置
test_angles = [45, 90, 90, 90]
test_pos, _, _ = forward_kin.forward_kinematics(test_angles)
print(f"測試角度 {test_angles} 的位置: {test_pos}")

# 嘗試逆解回去
angles2, success2, iter2 = inverse_kin.inverse_kinematics_iterative(test_pos, max_iterations=200)
print(f"逆解結果: 成功={success2}, 迭代={iter2}")
print(f"計算角度: {angles2}")
print(f"預期角度: {test_angles}\n")

print("=== 測試3: 測試大臂抬起的位置 ===")
test_angles3 = [90, 60, 90, 90]
test_pos3, _, _ = forward_kin.forward_kinematics(test_angles3)
print(f"測試角度 {test_angles3} 的位置: {test_pos3}")

angles3, success3, iter3 = inverse_kin.inverse_kinematics_iterative(test_pos3, max_iterations=200)
print(f"逆解結果: 成功={success3}, 迭代={iter3}")
print(f"計算角度: {angles3}")
