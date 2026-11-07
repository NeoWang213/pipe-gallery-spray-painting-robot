# 实物与演示素材索引

## 照片和图示

| 内容 | 文件 |
| --- | --- |
| 整机与个人工作展示 | [展示图](images/project-overview.jpg) |
| 机器人整体 | [整机照片](images/robot.jpg) |
| 麦克纳姆轮底盘 | [底盘照片](images/chassis.jpg) |
| 末端喷枪与相机装配 | [模型图](images/end-effector.png) |
| 喷枪电控平台 | [照片](images/controller.jpg) |
| 气动通路 | [照片](images/pneumatic.jpg)、[原始原理图](images/pneumatic-original.png) |
| 整理绘制的系统架构 | [PNG](images/system-architecture.png)、[SVG](images/system-architecture.svg) |
| 按描述重绘的电控示意 | [PNG](images/electrical-control.png)、[SVG](images/electrical-control.svg) |

## 原始视频

视频原文件保存在本地 `release-assets/`，供自行上传 Release 附件。预览为均匀抽帧，非连续实时录像，也不用于推算执行速度。

| 原文件 | 时长 | 预览 |
| --- | --- | --- |
| 完整客户端演示视频.mp4 | 约 30.17 秒 | [GIF](previews/client.gif) |
| 定位演示视频.mp4 | 约 32.72 秒 | [GIF](previews/positioning.gif) |
| 喷漆定位验证视频.mp4 | 约 22.00 秒 | [GIF](previews/painting.gif) |
| 蓝牙控制喷枪开关演示视频.mp4 | 约 6.83 秒 | [GIF](previews/bluetooth.gif) |

完整视频尺寸及字节数见 [媒体元数据](media-metadata.json)。本地还保留外购机械臂参考包 `xArm6(XF1305).zip`。

## 整理版离线界面

以下为本次整理版在无设备环境中的界面检查截图，区别于上面的历史实物演示。

- [v1.0 整机控制界面](images/curated-v1-offline.png)
- [v2.0 视觉规划界面](images/curated-v2-offline.png)
- [v2.0 往返扫描轨迹预览](images/curated-v2-path-preview.png)

## 来源和重新生成

英文 README 使用独立的 `-en` 展示图、架构图、电控图和动画字幕，并提供英文气动原理图。中文版继续使用中文素材。英文动画保留历史录屏和手机界面的原始文字，仅将后加字幕翻译为英文；原始视频未修改。英文图示可通过 `python tools/prepare_english_media.py` 重新生成。

照片及视频均来自用户提供的原始项目文件。展示图仅做缩放、拼版和文字标注，未生成或替换实物内容。视频原文件不重新编码。原始材料在工作目录保留，上传源码包包含展示用副本。

本地具备原始素材时，可使用 `python tools/prepare_media.py` 重建展示图与 GIF，使用 `python tools/draw_diagrams.py` 重建示意图。脚本依赖 Pillow / OpenCV 和 Windows 微软雅黑字体；源码上传包不包含视频原件，重新生成预览前需补入 `release-assets/`。
