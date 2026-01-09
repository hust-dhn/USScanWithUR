# 本文目录：
## 如何解决 SOCKS 相关问题
## 重新安装驱动
## 设置双相机模组
## github 快速 git pull
## 本地.git仓库使用
## 访问zzz服务器电脑（不用删除github关联的分支）
## Helper: pip+清华源

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
cd /home/zzz/ros2_ws/src/my_ur10e_control/linux_x86
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

# Helper: 本地.git仓库使用
## 步骤：
### 先确认你至少能到达这台主机（是否真的通）
ping -c 3 14.0.1.49
### 再做一次路由确认（看有没有走奇怪的网关/VPN）：
ip route get 14.0.1.49
### 确认本机有ssh客户端
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
### 检查
sudo ss -tlnp | grep ssh
看到 LISTEN 0 128 0.0.0.0:22 就成功了。
### 用“正确格式”的用户名
ssh zzz@14.0.1.49
### 创建裸仓库
mkdir -p ~/git-repos
cd ~/git-repos
git init --bare my_ur10e_control.git
### 检查
ls -la ~/git-repos/my_ur10e_control.git
### 本地仓库关联远程仓库并推送
git remote add intranet zzz@14.0.1.49:~/git-repos/my_ur10e_control.git
git push -u intranet --all
git push intranet --tags
### 验证
git remote -v
### 设置免密登录（可选，但推荐）
ssh-keygen -t ed25519
ssh-copy-id zzz@14.0.1.49
### 以后拉取和推送只需
git pull intranet
git push intranet

# Helper: 访问zzz服务器电脑（不用删除github关联的分支）
## 步骤：
### 如果先前没有克隆仓库（以后相当于只走内网）
git clone zzz@14.0.1.49:~/git-repos/my_ur10e_control.git
### 如果已经克隆过仓库
git remote -v
git remote add intranet zzz@14.0.1.49:~/git-repos/my_ur10e_control.git
git fetch intranet
git branch -vv
### 如果你工作分支是 master（你刚才推的是 master），直接把 upstream 指向内网：
git branch --set-upstream-to=intranet/master master
### 拉取和推送本地仓库只需
git fetch intranet
git pull intranet
git push intranet
### 拉取和推送github仓库
git fetch origin
git pull origin
git push origin
### 免密（推荐）
ssh-keygen -t ed25519
ssh-copy-id zzz@14.0.1.49

# Helper: pip+清华源
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple xxx

# Helper: 

# Helper: xxxx