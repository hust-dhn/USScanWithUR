import os
import glob

def delete_jpg_files(folder_path):
    # 获取所有 jpg 文件
    jpg_files = glob.glob(os.path.join(folder_path, "*.jpg"))
    
    # 删除每个 jpg 文件
    for file in jpg_files:
        try:
            os.remove(file)
            print(f"已删除文件: {file}")
        except Exception as e:
            print(f"删除文件 {file} 失败: {e}")

# # 使用示例
# folder_path = "/path/to/your/folder"  # 替换为目标文件夹路径
# delete_jpg_files(folder_path)
