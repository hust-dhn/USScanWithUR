# 本文目录：
## 如何解决 SOCKS 相关问题
## 重新安装驱动
## 设置双相机模组
## github 快速 git pull

# Helper: 如何解决 SOCKS 相关问题
## 背景说明
在使用 `pip` 或其他网络工具时，您可能会遇到与 SOCKS 代理相关的问题。通常，这种问题出现在通过代理连接网络时，或者由于环境配置不当导致无法正常访问网络资源。此文档提供了有关如何解决 SOCKS 相关问题的解决步骤。
## 常见问题
### 错误信息示例：
- `ERROR: Could not install packages due to an OSError: Missing dependencies for SOCKS support.`
- `WARNING: There was an error checking the latest version of pip.`
这些错误通常发生在您在网络上使用了代理，或者某些环境设置依赖于 SOCKS 协议（如 PySocks 库）。
## 解决步骤
### 1.进入网络设置：
在您提供的界面中，点击 网络代理 部分旁边的设置图标。
### 2.将代理设置为自动：
在弹出的设置窗口中，将 网络代理 设置为 自动。这将允许系统根据网络环境自动选择代理设置。

# Helper: 重新安装驱动
## 步骤：
cd /home/zzz/ros2_ws/src/my_ur10e_control/camera/linux_x86
sudo dpkg -i inudev_4.36.0008.02-1_amd64.deb

# Helper: 设置双相机模组
## 步骤：
### 打开opt下的解压文件
cd /opt/
cd /opt/inuitive
cd /opt/inuitive/InDev/
### 复制两份InuService
cd ../bin/       
sudo cp -r InuService InuService1
sudo cp -r InuService InuService2
### 复制两份InuServiceParams.xml
cd ../config/
sudo cp -r InuServiceParams.xml InuService1Params.xml
sudo cp -r InuServiceParams.xml InuService2Params.xml
### 分别编辑InuService1Params.xml与InuService2Params.xml
sudo vi InuService1Params.xml       //将第一行的InuServiceParams与最后一行的InuServiceParams改为InuService1Params，保存后退出
sudo vi InuService2Params.xml       //将第一行的InuServiceParams与最后一行的InuServiceParams改为InuService2Params，保存后退出
### 检查USB接口是否识别两个相机
lsusb/lsusb -t
### 上述步骤完成后打开两个终端分别输入
sudo ./InuService1 console 1
sudo ./InuService2 console 2         //若均显示Module is ready to work，则说明模组正确连接

# Helper: github 快速 git pull
## 步骤：
### 并行fetch、清理无用引用
git config --global fetch.parallel 8
git config --global fetch.prune true
git remote prune origin
### 快速拉取最新代码
git fetch origin   // 或者如下
git pull

# Helper: xxxx

