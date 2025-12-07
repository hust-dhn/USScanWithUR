import usb.core
import usb.util
import cv2
import numpy as np

# 1. 查找设备
dev = usb.core.find(idVendor=0x2959, idProduct=0x4001)
if dev is None:
    raise ValueError('设备未找到，请检查连接。')

print(f"找到设备: {dev}")

# 2. 探测设备配置和端点（关键步骤）
dev.set_configuration()
cfg = dev.get_active_configuration()
intf = cfg[(0,0)]  # 获取第一个接口

print("发现端点:")
for ep in intf:
    print(f"  端点地址: 0x{ep.bEndpointAddress:02x}, 属性: {ep.bmAttributes}")

# 3. 尝试从批量传输端点读取数据
# 注意：你需要根据上一步打印的端点信息，替换下面的 `bulk_in_ep` 地址。
# 通常，图像数据通过 `IN` 端点（地址最高位为1）传输。
try:
    # 假设我们找到了一个批量输入端点，例如 0x81 或 0x83
    bulk_in_ep = 0x81  # !!! 请根据实际探测结果修改这个值 !!!
    
    # 读取更大块的数据（例如 64KB），因为一帧图像可能很大
    print(f"尝试从端点 0x{bulk_in_ep:02x} 读取数据...")
    data = dev.read(bulk_in_ep, 65536, timeout=5000)  # 超时5秒
    print(f"成功读取 {len(data)} 字节的原始数据。")
    
    # 4. 尝试解析和显示图像
    # 方法A: 尝试解码为JPEG（如果相机输出JPEG格式）
    try:
        # 将字节数据转换为numpy数组
        nparr = np.frombuffer(data.tobytes(), np.uint8)
        frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow('Image (JPEG Decoded)', frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            print("已使用JPEG解码方式显示图像。")
        else:
            print("JPEG解码失败，数据可能不是JPEG格式。")
    except Exception as e:
        print(f"JPEG解码出错: {e}")
    
    # 方法B: 如果方法A失败，尝试将原始数据当作RAW图像处理
    # 这需要你知道图像的宽度、高度和像素格式。这里只是一个示例。
    # 假设是 640x480 的灰度图 (单通道，8位)
    # width, height = 640, 480
    # if len(data) >= width * height:
    #     raw_frame = np.array(data[:width*height], dtype=np.uint8).reshape((height, width))
    #     cv2.imshow('Image (RAW Grayscale)', raw_frame)
    #     cv2.waitKey(0)
    
except usb.core.USBError as e:
    print(f"USB通信错误: {e}")
except Exception as e:
    print(f"发生错误: {e}")
finally:
    # 5. 清理
    usb.util.dispose_resources(dev)
    print("资源已清理。")