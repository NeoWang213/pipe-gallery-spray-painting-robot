# 自行上传 GitHub

本次交付仅保存在本地，未创建或发布 GitHub 仓库。

建议仓库：`NeoWang213/pipe-gallery-spray-painting-robot`，公开，描述可填写：

> Compact robot for pipe-gallery maintenance with a custom spray system, Python desktop control, and vision-guided painting path planning. Includes SolidWorks models and physical demonstrations.

可直接复制 [英文仓库描述](repository-description.txt)。仓库首页默认显示英文 `README.md`，通过顶部语言链接切换至中文 `README.zh-CN.md`。

## 源码与文档

可直接使用工作目录旁的 `pipe-gallery-spray-painting-robot-upload/`，或解压同名 ZIP。该目录已排除虚拟环境、运行日志、大视频与原始重复文件。

方式一：在 GitHub 网页新建空仓库，选择“Add file → Upload files”，上传**解压后的目录内容**。上传 ZIP 只会得到一个 ZIP 文件，不会自动展开为仓库。

方式二：在上传目录执行：

```powershell
git init -b main
git add .
git commit -m "Add spray-painting robot project portfolio"
git remote add origin https://github.com/NeoWang213/pipe-gallery-spray-painting-robot.git
git push -u origin main
```

提交前按自己的账号配置 Git 作者信息，并使用自己的 GitHub 认证。若新建仓库时已生成 README，先克隆远端并复制文件到克隆目录，再提交推送；不要覆盖远端历史。

## 完整视频与参考模型

四段原始视频保存在工作目录 `release-assets/`。其中喷漆定位验证视频约 114 MiB，超过普通 Git 单文件限制，应作为 **Release 附件**上传。

1. 推送代码后，在 Releases 中建立项目档案发布，标签可用 `v0.1.0-portfolio`。
2. 标题填写“项目作品集与原始演示材料”，正文使用 [release-notes.md](release-notes.md)。
3. 将四个原始 `.mp4` 作为附件上传，文件名与 [素材索引](gallery.md) 对应。
4. `xArm6(XF1305).zip` 是第三方模型参考包，在确认其分发许可后再决定是否公开上传。
5. 发布后可在 README 的“实物演示”末尾补充该 Release 的实际页面链接。

原始视频与参考包未进入源码 ZIP。`release-assets/` 和顶层原视频已加入 `.gitignore`。

## 上传后检查

打开仓库首页确认图片和四个动画能显示；点击用户手册、软件说明及 SolidWorks 模型目录确认文件可达；检查代码目录中没有 `.venv`、日志或超过限制的文件。当前未附 LICENSE，符合暂不指定开源许可的约定。
