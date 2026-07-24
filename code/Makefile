# 定义 Gurobi 路径 
GUROBI_HOME = /opt/gurobi1301/linux64
GUROBI_INC  = -I$(GUROBI_HOME)/include
GUROBI_LIB  = -L$(GUROBI_HOME)/lib -lgurobi_c++ -lgurobi130 -lm

# 编译器参数
CXX = g++
CXXFLAGS = -O3 -std=c++11 -Wall $(GUROBI_INC)

# 自动获取当前目录下所有的 .cpp 文件并转换为 .o 文件
SRCS = $(wildcard *.cpp)
OBJS = $(SRCS:.cpp=.o)

# 将目标文件重新命名为 ilrp，与你习惯的运行命令保持一致
TARGET = ilrp

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(GUROBI_LIB)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 强化清理逻辑：一并删除可能遗留的旧版 ilrp 和 之前错误生成的 CLRP_Solver
clean:
	rm -f *.o $(TARGET) CLRP_Solver