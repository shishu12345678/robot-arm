# GCR3-618 MDH 多姿态公式验证

本仓库提供只读验证：将静止采集的实际关节角代入两台机器人各自的标定 MDH 正运动学公式，再与同次采集的控制器法兰位姿比较。工具不连接机器人、不包含使能、停止或运动调用。

## 模型约定

每节采用

\[
A_i=T_x(a_i)R_x(\alpha_i)R_z(q_i+\theta_{0,i})T_z(d_i),\qquad
{}^BT_F=\prod_{i=1}^{6}A_i.
\]

参数来自控制器 `get_calib_DH`，参数次序为 `a, alpha, d, theta0`，长度单位 m、角度单位 rad。配置文件分别固定左右臂标定参数；验证程序不会从每条样本覆盖模型参数。SDK 返回的法兰姿态按固定 XYZ RPY 解释：`R=Rz(yaw)Ry(pitch)Rx(roll)`。

本程序只比较每台机械臂自身基坐标系下的法兰。它不验证双臂世界系外参、工具 TCP、实体外壳碰撞或外部测量得到的绝对精度。

## 准备采样

在厂家 RPC 样例环境中运行已有的只读采集程序。每个 JSON 应含 `get_actual_joints_position`、`get_flange_pose`、`target`、`collected_at_utc` 和注明 `read_only` 的 `note`。每个姿态应停稳后再采集。请备份现有文件，并把每次采样另存为独立 JSON；旧版采集器会覆盖固定文件名，左臂的目标标签也要改为 `left_slave_arm`。

实际采样应覆盖各关节变化及组合姿态；不要只改变 J1。工具不规定机械臂运动范围或通过阈值，阈值应在实验方案中依据测量分辨率预先确定。

## 运行

仅需 Python 3，无第三方依赖。把采样放在 `data/right/` 或 `data/left/`，从仓库根目录运行：

```powershell
python mdh_validate.py data/right --arm right_main_arm
python mdh_validate.py data/left --arm left_slave_arm
```

也可以传入单个 JSON 文件。结果写入 `results/`：CSV 逐姿态列出关节角、法兰 XYZ 差、位置误差模长和旋转角误差；JSON 同时包含最大值与 RMS。程序报告数值，不自动判定通过。

## 本地测试

```powershell
python -m unittest discover -s tests -v
```

只提交去除控制器 IP、序列号等设备标识后的采样数据；原始采样默认不纳入版本控制。
