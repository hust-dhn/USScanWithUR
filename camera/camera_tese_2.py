import usb.core
import usb.util

# 查找设备，替换为你设备的 Vendor ID 和 Product ID
dev = usb.core.find(idVendor=0x2959, idProduct=0x4001)

if dev is None:
    raise ValueError('Device not found')

# 设置设备配置
dev.set_configuration()
print(dev)

# 向设备发送控制请求或者读取数据
# 具体的控制请求取决于相机的协议，通常你可以使用如下的命令：
# 发送控制请求（例如请求数据）:
# dev.ctrl_transfer(0x40, 0x01, 0, 0, 'data')

# 可以使用相机提供的API与设备进行更多交互
# 读取图像数据
# 假设设备通过批量传输传输图像数据
data = dev.read(0x81, 1024)  # 0x81 是批量传输端点，1024 是一次读取的字节数
import cv2
import numpy as np

# 假设 data 是从设备读取的原始图像数据
# 根据图像格式，将字节数据转换为 OpenCV 图像格式
frame = np.array(data, dtype=np.uint8)

# 显示图像
cv2.imshow('Image', frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
