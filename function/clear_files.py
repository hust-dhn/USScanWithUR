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

if __name__ == "__main__":

    # folder_path = "camera/calib_output/"  # 替换为目标文件夹路径
    # delete_jpg_files(folder_path)

    # folder_path_debug_imgs = "camera/calib_output/debug_images/"  # 替换为目标文件夹路径
    # delete_jpg_files(folder_path_debug_imgs)

    # folder_path_lc_imgs = "camera/lc_imgs/"  # 替换为目标文件夹路径
    # delete_jpg_files(folder_path_lc_imgs)

    # folder_path_rc_imgs = "camera/rc_imgs/"  # 替换为目标文件夹路径
    # delete_jpg_files(folder_path_rc_imgs)

    folder_path_left_imgs = "camera/stereo_imgs/lc_imgs/"  # 替换为目标文件夹路径
    delete_jpg_files(folder_path_left_imgs)

    folder_path_right_imgs = "camera/stereo_imgs/rc_imgs/"  # 替换为目标文件夹路径
    delete_jpg_files(folder_path_right_imgs)
