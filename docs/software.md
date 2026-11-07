# 软件运行与接口说明

## 环境与入口

建议 Windows 10 / 11、64 位 Python 3.10。v1.0 与 v2.0 使用不同 Qt 框架，分别以独立 Python 进程启动，不在同一进程中混用。

```powershell
python -m venv .venv
# 选择需要的版本安装依赖
.\.venv\Scripts\python.exe -m pip install -r requirements-v1.txt
.\.venv\Scripts\python.exe -m pip install -r requirements-v2.txt

.\.venv\Scripts\python.exe run.py --version v1 --offline
.\.venv\Scripts\python.exe run.py --version v2 --offline
```

`run.py` 从仓库定位配置与运行目录，不依赖终端当前工作目录。`--config config/local.json` 可加载自己的设置，嵌套配置段按整段替换；建议复制完整模板后修改。日志与 v1 标定文件位于自动建立的 `runtime/`，不进入 Git。

原始用户手册保留在 [user-manual-v1.pdf](user-manual-v1.pdf)，描述历史 v1.0。整理版新增或修复的行为以本文为准。

## v1.0：整机串口控制

功能包含双路视频、底盘四方向遥控、六关节滑块、选区面积计算及作业参数下发。使用 `config/settings.json` 中的相机编号和串口波特率。

实物使用步骤：

1. 连接末端与底盘摄像头，核对 `camera_ids.arm` 和 `camera_ids.chassis`。本机设备编号可能不同。
2. 执行 `python run.py --version v1`，选择实际串口并连接。默认波特率为原代码的 `115200`。
3. 面积计算需要有效的 `runtime/camera_calibration.yaml`，包含实测 `camera_matrix`、`dist_coeffs`、`T_cam_to_base`。整理版缺少文件时不生成替代标定，也不显示默认矩阵算出的面积。
4. 内参可通过棋盘格图像计算；棋盘格尺寸与图像分辨率应与实物一致。**相机内参标定不等于手眼标定**，外参需独立实测。平面模型的平面位于基座坐标的 `z=0`，外参平移量按米使用，面积单位为平方米。
5. 通过遥控按钮或关节滑块下发指令；下位机必须实现对应协议。当前材料没有下位机固件，无法仅凭本仓库复现整机通信闭环。

串口协议按原 Python 文件整理，尚未用下位机重新核验：

| 字段 | 编码 |
| --- | --- |
| 帧头 | `0xAA` |
| 命令 | `0x01` 六关节；`0x02` 底盘；`0x03` 作业参数 |
| 长度 | 1 字节，载荷长度 |
| 载荷 | 小端：关节 `<6f`；底盘 `<3f`；作业参数 `<fHf` |
| 校验 | 前述字节和的低 8 位 |
| 帧尾 | `0x55` |

底盘三个浮点数对应原代码的平移与转动指令。原文件中速度标注存在 RPM / mm/s 的不一致，作业参数的最终含义和量纲须与下位机核对。厚度是下发参数，现有资料未提供涂层厚度闭环测量。

v1.0 的“紧急停止”按钮在代码中断开串口；是否停止执行取决于下位机实现。整理版明确显示这一行为。距离栏显示未接入反馈，避免将历史模拟值误认为实测数据。关节状态是界面指令值，不是编码器回读。

## v2.0：视觉轨迹规划与网口执行

### 离线查看

```powershell
.\.venv\Scripts\python.exe run.py --version v2 --offline
```

依次点击黄色框内矩形的两个对角点，调整像素间距，点击“预览轨迹”或“执行轨迹”。可查看往返扫描路径；离线确认后仅记录预览结果。圆形模式依次选择圆心和圆周点，预览同心圆路径。

界面中的轨迹间距单位是 **px**，尚未建立它与喷幅、重叠率或涂层厚度的实验关系。

### 接入实物

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-hardware-v2.txt
.\.venv\Scripts\python.exe run.py --version v2
```

1. 安装相机驱动并连接 RealSense，使用 640 × 480、30 FPS 的彩色与深度流，深度对齐到彩色图。型号需要依据实物确认，原始代码中的 D435 字样不单独作为型号证据。
2. 通过网线连接 xArm，确保电脑与机械臂网络可达。修改配置中的 `arm_ip`，也可在界面输入。
3. 按实际相机、喷枪装配和当前机械臂观察位姿核对坐标映射、初始位姿。确认后在本地配置设置 `mapping_reviewed: true`。默认值来自历史程序，尚无本次实物复测。
4. 底盘保持固定，机械臂先到合适的观察位置，选取区域并生成预览。三维模式要求当前有效深度帧；缺帧、超过 2 秒的缓存帧、非有限值、零深度或超出配置范围的轨迹点都会使整条路径生成失败。
5. 确认后在后台线程逐点执行，界面保持响应。停止、失败和断连分别记录，不显示为任务完成。喷枪由独立控制器开关，上位机没有喷枪自动联动接口。

相机在采集线程中读取，界面从缓存取得深度帧。确认预览到执行之间应保持目标和观察位姿不变；任务不进行在线重规划或目标跟踪。

### 坐标映射与面积

针孔反投影：`Xc=(u-cx)·Z/fx`，`Yc=(v-cy)·Z/fy`，深度 `Z` 使用毫米。历史三维映射为：

```text
Xa = origin_x_mm + scale_x * Xc + offset_x_mm
Ya = scale_y * Yc + offset_y_mm
Za = Z + offset_z_mm
```

这是保留下来的**实验经验映射**，并非完整的随机械臂姿态更新的手眼变换。修改观察姿态或安装方式后需要重新验证映射。喷枪朝向采用配置的固定姿态，不包含沿管道曲率调整法向的功能。

二维模式按固定毫米/像素比例映射；超出原实验范围时拒绝整条执行，不通过坐标裁剪改变路径。三维模式不会在深度缺失时自动切换二维，也不会使用默认 500 mm 或邻域平均值替代无效轨迹点。

整理版的面积标签为**平面投影面积估计**：以轨迹采样的平均深度、相机焦距及选区像素面积估计 `A=Apixel·Z²/(fx·fy)`，毫米平方换算为平方米。这不是管道曲面展开面积，也没有提供误差精度指标。离线模式不显示实际面积数值。

## 检查命令

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m compileall -q software run.py
$env:QT_QPA_PLATFORM = 'offscreen'
.\.venv\Scripts\python.exe run.py --version v1 --smoke-test
.\.venv\Scripts\python.exe run.py --version v2 --smoke-test
```

硬件依赖仅在实际接入时需要；`--offline` 不建立物理连接。界面检查和模拟控制器测试不能代替实物复测。
