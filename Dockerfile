# Container image that runs your code
FROM raiots/mass_env:amd64

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY . /MASS/

# 设置工作目录
WORKDIR /MASS

# 安装Python依赖并编译
RUN virtualenv /MASS/venv --python=python3 && \
    . /MASS/venv/bin/activate && \
    pip install torch torchvision numpy matplotlib ipython && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j8

# 设置Python脚本运行命令
CMD cd /MASS/python && . /MASS/venv/bin/activate && python3 main.py -d ../data/metadata.txt