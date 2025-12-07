import usb.core
import usb.util

dev = usb.core.find(idVendor=0x2959, idProduct=0x4001)
if dev is None:
    raise ValueError('Device not found')
dev.set_configuration()

print("=== 基于有效指令进行针对性探测 ===\n")
# 已知有效的参数：bmRequestType=0xC0, bRequest=0x0C
bmRequestType = 0xC0
bRequest = 0x0C

# 尝试不同的 wIndex (通常 0x00是接口，0x81可能是端点地址)
for wIndex in [0x0000, 0x0001, 0x0081]:
    # 尝试不同的 wValue (可能代表子命令或索引)
    for wValue in [0x0000, 0x0001, 0x1000, 0x2000]:
        try:
            print(f"尝试: Type=0x{bmRequestType:02X}, Req=0x{bRequest:02X}, Val=0x{wValue:04X}, Idx=0x{wIndex:04X}")
            # 请求少量数据
            data = dev.ctrl_transfer(bmRequestType, bRequest, wValue, wIndex, 64, timeout=1500)
            if len(data) > 0:
                hex_str = data.tobytes().hex()
                ascii_str = ''.join([chr(c) if 32 <= c < 127 else '.' for c in data])
                print(f"  成功! 返回 {len(data)} 字节")
                print(f"  十六进制: {hex_str[:80]}...")
                print(f"  ASCII可读部分: {ascii_str[:40]}")
            else:
                print(f"  成功，返回0字节。")
        except usb.core.USBError as e:
            print(f"  失败: {e}")

usb.util.dispose_resources(dev)
print("\n探测结束。")