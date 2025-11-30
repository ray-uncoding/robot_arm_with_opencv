#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
分析機械手臂運動學特性
"""

import numpy as np
from kinematics_control import DHParameters, ForwardKinematics

dh_params = DHParameters()
forward_kin = ForwardKinematics(dh_params)

print("=== DH參數表 ===")
print("關節  theta(初始)   d      a    alpha")
for i in range(5):
    print(f"  {i+1}     {dh_params.dh_table[i, 0]:6.1f}  {dh_params.dh_table[i, 1]:5.0f}  {dh_params.dh_table[i, 2]:5.0f}  {dh_params.dh_table[i, 3]:6.1f}")

print("\n=== 測試不同伺服角度的末端位置 ===")
test_cases = [
    ([90, 90, 90, 90], "初始位置"),
    ([90, 60, 90, 90], "大臂上抬30度"),
    ([90, 120, 90, 90], "大臂下壓30度"),
    ([90, 90, 60, 90], "小臂上抬30度"),
    ([90, 90, 120, 90], "小臂下壓30度"),
    ([45, 90, 90, 90], "底座轉45度"),
    ([135, 90, 90, 90], "底座轉135度"),
]

for angles, desc in test_cases:
    pos, _, _ = forward_kin.forward_kinematics(angles)
    r = np.sqrt(pos[0]**2 + pos[1]**2)
    print(f"{desc:20s} 角度{angles} -> X={pos[0]:7.1f}, Y={pos[1]:7.1f}, Z={pos[2]:7.1f}, R={r:7.1f}")
