from pathlib import Path
import shutil

src = Path("camera/lc_imgs")       # 源文件夹
dst = Path("camera/lc_imgs_1")     # 目标文件夹
dst.mkdir(parents=True, exist_ok=True)

count = 0
for p in src.glob("*_1.jpg"):
    shutil.copy2(p, dst / p.name)  # copy2 会保留修改时间等元信息
    count += 1

print(f"Copied {count} files to {dst}")
