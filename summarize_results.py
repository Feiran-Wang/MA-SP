#!/usr/bin/env python3
import os
import re
import csv

# ==========================================
# 1. 配置 
# ==========================================
RESULTS_DIR = "./results"
NUM_RUNS = 20  # 对应种子 0 到 19

# 新增开关：默认关闭 (False)。
# 当设为 True 时，不在终端打印表格，而是将结果保存为 CSV 文件。
SAVE_AS_CSV = True
CSV_FILENAME = "results_summary.csv"

# ==========================================
# 2. 动态获取算例列表与自定义排序
# ==========================================
instances = set()
if os.path.exists(RESULTS_DIR):
    for filename in os.listdir(RESULTS_DIR):
        # 匹配诸如 "500-25-2e_6.txt" 的文件，组1为算例名，组2为种子号
        match = re.match(r"^(.*)_(\d+)\.txt$", filename)
        if match:
            instances.add(match.group(1))

# 自定义排序逻辑函数
def instance_sort_key(inst):
    parts = inst.split('-')
    try:
        # 将前两部分（如 500 和 25）转换为整数，实现数值排序（5 会排在 10 前面）
        # 将最后一部分（如 1e, 2a）保留为字符串，实现按位字典序排序
        return (int(parts[0]), int(parts[1]), "-".join(parts[2:]))
    except (ValueError, IndexError):
        # 如果遇到不符合 "数字-数字-字符串" 格式的异常算例名，退化为普通字符串兜底处理
        return (0, 0, inst)

# 将集合转换为有序列表，并应用自定义排序规则
instances = sorted(list(instances), key=instance_sort_key)

# ==========================================
# 3. 解析与计算
# ==========================================
output_data = []

for inst in instances:
    objs = []
    times = []

    # range(NUM_RUNS) 生成 0 到 19，匹配 _0.txt 到 _19.txt
    for seed in range(NUM_RUNS):
        file_path = os.path.join(RESULTS_DIR, f"{inst}_{seed}.txt")
        
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                content = f.read()
                
                # 提取 Objective 数值
                obj_match = re.search(r"The objective is:\s+([\d\.e\+]+)", content)
                if obj_match:
                    try:
                        objs.append(float(obj_match.group(1)))
                    except ValueError:
                        pass
                
                # 提取运行时间
                time_match = re.search(r"The running time is:\s+([\d\.e\+]+)", content)
                if time_match:
                    try:
                        times.append(float(time_match.group(1)))
                    except ValueError:
                        pass

    if objs:
        best_obj = min(objs)
        avg_obj = sum(objs) / len(objs)
        avg_time = sum(times) / len(times) if times else 0
        
        # 计算当前进度
        progress_str = f"{len(objs)}/{NUM_RUNS}"
        
        if SAVE_AS_CSV:
            # 存入列表，方便后续写入 CSV
            output_data.append([inst, f"{best_obj:.2f}", f"{avg_obj:.2f}", f"{avg_time:.2f}", progress_str])
        else:
            # 兼容终端打印格式
            count_str = f"({progress_str})" if len(objs) < NUM_RUNS else ""
            output_data.append(f"{inst:<15} | {best_obj:<12.2f} | {avg_obj:<12.2f} | {avg_time:<12.2f} {count_str}")
    else:
        # 如果没找到匹配文件，跳过
        pass

# ==========================================
# 4. 结果输出
# ==========================================
if SAVE_AS_CSV:
    headers = ["Instance", "Best Obj", "Avg Obj", "Avg Time (s)", "Progress"]
    with open(CSV_FILENAME, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(output_data)
    print(f"数据已成功保存至 {CSV_FILENAME}")
else:
    print(f"{'Instance':<15} | {'Best Obj':<12} | {'Avg Obj':<12} | {'Avg Time (s)':<12}")
    print("-" * 65)
    for line in output_data:
        print(line)