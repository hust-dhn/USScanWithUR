in_path = "camera/robot_pos.txt"
out_path = "camera/robot_pos_1.txt"

with open(in_path, "r", encoding="utf-8") as f_in, open(out_path, "w", encoding="utf-8") as f_out:
    for i, line in enumerate(f_in, start=1):   # i 从 1 开始
        if i % 3 == 2:                         # 2,5,8,...
            f_out.write(line)

print("done")
