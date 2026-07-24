#!/bin/bash

# ==========================================
# 0. 全局实验参数 
# ==========================================
TIME_LIMIT=60000       # [参数6] 时间限制 (秒)
MAX_ITER=300000        # [参数7] 最大迭代次数
NUM_RUNS=20            # 重复次数 
MAX_JOBS=20            # 批量任务数

# ==========================================
# 1. 路径与环境配置
# ==========================================
# 算例根目录
INSTANCE_DIR="./instances/set_S"
# Depot 配置目录 (注意：路径中含有空格，脚本中已做双引号处理)
DEPOT_DIR="./instances/depot configurations/S"
RESULT_DIR="./results"
LOG_DIR="./logs"

export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1

mkdir -p "$RESULT_DIR" "$LOG_DIR"

# 需要运行的算例列表
INSTANCES=(
     "all"
    # "500-30-1b"
    # "300-20-1e" 
    # "400-20-4e" 
    # "400-25-3e" 
    # "500-25-2e"
    # "500-25-3e"
    # "500-30-2a" 
    # "500-30-2b" 
    # "500-30-3a" 
    # "500-30-4a" 
    # "500-30-4b" 
    # "500-30-4e"
    # "600-30-2a" 
    "600-30-3a" 
    "600-30-3b" 
    "600-30-3c"
    "600-30-3d"  
    "600-30-3e" 
    # "600-30-4e" 
)

# ==========================================
# 1.5 解析全算例模式 (新增逻辑)
# ==========================================
ACTUAL_INSTANCES=()

for inst in "${INSTANCES[@]}"; do
    if [ "$inst" == "all" ]; then
        echo "检测到 'all' 模式，正在读取 $INSTANCE_DIR 下的所有算例..."
        # 遍历目录下所有 json 文件
        for file in "$INSTANCE_DIR"/*.json; do
            if [ -f "$file" ]; then
                # 提取纯文件名（不含路径）
                filename=$(basename "$file")
                # 去除 .json 后缀，压入实际运行队列
                ACTUAL_INSTANCES+=("${filename%.json}")
            fi
        done
    else
        # 非 all 模式下的指定算例直接压入队列
        ACTUAL_INSTANCES+=("$inst")
    fi
done

# 去重（防止用户同时写了 "all" 和具体的算例导致重复跑）
ACTUAL_INSTANCES=($(echo "${ACTUAL_INSTANCES[@]}" | tr ' ' '\n' | sort -u | tr '\n' ' '))

echo "共计检测到 ${#ACTUAL_INSTANCES[@]} 个待运行算例。"

# ==========================================
# 2. 核心运行函数 
# ==========================================
run_instance() {
    local inst=$1
    local seed=$2
    local out_file=$3
    
    # 构造参数路径
    local inst_path="${INSTANCE_DIR}/${inst}.json"
    # 根据您的示例，Depot 文件名包含 "depot_configuratons_" 前缀 (注意原文字符串拼写)
    local depot_path="${DEPOT_DIR}/depot_configuratons_${inst}.txt"
    local log_file="${LOG_DIR}/${inst}_${seed}.log"
    
    # [参数说明对照表]
    # $1: $inst_path   (算例路径)
    # $2: $depot_path  (Depot配置路径)
    # $3: $out_file    (输出路径)
    # $4: 0            (文献最优值，默认设为0)
    # $5: 0            (是否最优标志)
    # $6: $TIME_LIMIT  (时间限制)
    # $7: $MAX_ITER    (最大迭代)
    # $8: $seed        (随机种子)

    echo "[$(date +'%H:%M:%S')] 启动: $inst | 种子: $seed"

    # 执行命令，使用双引号包裹变量以处理路径中的空格
    ./code/ilrp "$inst_path" "$depot_path" "$out_file" 0 0 "$TIME_LIMIT" "$MAX_ITER" "$seed" > "$log_file" 2>&1
    
    if [ $? -ne 0 ]; then
        echo "Error: $inst seed $seed failed." >> failed_instances.log
    fi
}

# ==========================================
# 3. 任务分发逻辑
# ==========================================
FIFO_FILE="/tmp/ilrp_queue_$$.fifo"
mkfifo "$FIFO_FILE"
exec 6<>"$FIFO_FILE"
rm -f "$FIFO_FILE"
for ((i=0; i<MAX_JOBS; i++)); do echo >&6; done

# 注意这里将循环数组改为了 ACTUAL_INSTANCES
for inst in "${ACTUAL_INSTANCES[@]}"; do
    for ((seed=0; seed<NUM_RUNS; seed++)); do
        
        # 结果文件名：例如 results/600-30-2a_0.txt
        out_file="${RESULT_DIR}/${inst}_${seed}.txt"
        
        # 检查文件是否已存在（跳过逻辑）
        if [ -f "$out_file" ]; then
            echo "跳过: $inst (种子 $seed) [已存在]"
            continue
        fi
        
        read -u6
        {
            run_instance "$inst" "$seed" "$out_file"
            echo >&6
        } &
    done
done

wait
exec 6>&-
echo "所有实验任务处理完成。"