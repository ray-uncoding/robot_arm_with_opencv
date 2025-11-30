#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
調試正向運動學
"""

import numpy as np
from kinematics_control import DHParameters, ForwardKinematics

# 初始化
dh_params = DHParameters()
forward_kin = ForwardKinematics(dh_params)

# 測試伺服馬達在90度時的位置
servo_angles = [90, 90, 90, 90]
print(f"伺服馬達角度: {servo_angles}")

# 顯示DH參數轉換
dh_current = dh_params.get_dh_params(servo_angles)
print(f"\nDH參數表 (轉換後):")
print("關節  theta      d      a    alpha")
for i in range(len(dh_current)):
    print(f"  {i+1}   {dh_current[i, 0]:7.1f} {dh_current[i, 1]:6.1f} {dh_current[i, 2]:6.1f} {dh_current[i, 3]:7.1f}")

# 計算末端位置
end_pos, _, transforms = forward_kin.forward_kinematics(servo_angles)
print(f"\n末端效應器位置: [{end_pos[0]:.1f}, {end_pos[1]:.1f}, {end_pos[2]:.1f}]")

# 顯示所有關節位置
print(f"\n所有關節位置:")
for i, T in enumerate(transforms):
    pos = T[:3, 3]
    print(f"關節{i}: [{pos[0]:7.1f}, {pos[1]:7.1f}, {pos[2]:7.1f}]")
