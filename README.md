# RIOT-experiment

## 1 介绍
物理网课程实验，需要提前下载[RIOT](https://github.com/RIOT-OS/RIOT)， 将这个仓库里面的代码替换加入RIOT的example里面

## 2 RIOT环境安装
本实验最好是在Ubuntu20.04系统下进行，建议准备一个虚拟机，虚拟机安装和搭建Ubuntu20.04系统的具体教程可以参考[这篇文章](https://zhuanlan.zhihu.com/p/355314438)，我们也会提供已经测试好的虚拟机环境包(预计8G)。

### 2.1 RIOT项目拉取
需要去github上下载RIOT开源项目，由于我们后续使用的开发板是ESP32系列，需要安装ESP32编译工具。
```bash
cd ~
git clone https://github.com/RIOT-OS/RIOT
# 可用国内镜像源，如 git clone https://githubfast.com/RIOT-OS/RIOT
```
如果有什么问题，可以先去浏览下[官网文档](https://doc.riot-os.org/getting-started.html)。

RIOT系统需要额外安装以下工具：
- Essential system development tools (GNU Make GCC, standard C library headers)
- git
- GDB in the multiarch variant (alternatively: install for each architecture you target the corresponding GDB package)
- unzip or p7zip
- wget or curl
- python3
- pyserial (linux distro package often named python3-serial or py3-serial)
- Doxygen for building the documentation

以Ubuntu为类，执行以下一个指令即可完成以上工具的安装。

```bash
sudo apt install -y git gcc-arm-none-eabi make gcc-multilib \
    libstdc++-arm-none-eabi-newlib openocd gdb-multiarch doxygen \
    wget unzip python3-serial
```

### 2.2 下载ESP32编译工具
这里提供两种方法，一种是esp32工具链直接下载在主机环境，另外一种方法是下载已编译好的容器环境，这两种方法都可行，二选一即可，后续编译项目需要记住对应的编译方法。

#### 2.2.1 方法一：本地工具链下载
要编译ESP32能使用的代码需要安装ESP32的工具链， RIOT代码里面提供了下载ESP32工具链的脚本，并且加入工具链的环境变量，执行以下四个命令即可完成。如遇到问题，可先浏览[官网RIOT esp32工具链安装文档](https://doc.riot-os.org/group__cpu__esp32.html#esp32_local_toolchain_installation)。

```bash
cd ~/RIOT
# 国内镜像
export IDF_GITHUB_ASSETS="dl.espressif.cn/github_assets"
dist/tools/esptools/install.sh all
echo 'alias esp_idf=". ~/RIOT/dist/tools/esptools/export.sh"' >> ~/.bashrc  
source ~/.bashrc
```

之后，去安装好的环境进行测试，这里使用最简单的hello-word的案例。使用板子为esp32-wroom-32，通过USB连接到电脑，并给用户提供读写设备端口的权限（每次重新连接USB都要执行）。
> 在Linux系统中，ESP32连接到计算机时，通常会被识别为/dev/ttyUSB0或类似的串口设备。如果用户没有权限访问/dev/ttyUSB0，通常是因为缺少相应的权限。将用户添加到dialout组：在Linux系统中，通常/dev/ttyUSB0的权限属于dialout组。你可以将用户添加到dialout组中，以获得访问权限。
```bash
# 运行以下命令，将当前用户添加到dialout组：
sudo usermod -a -G dialout $USER
# 重新加载用户组
newgrp dialout
# chmod 只能临时更改权限
# sudo chmod 777 /dev/ttyUSB*   
# 每次带开命令行都要执行
esp_idf all
cd ~/RIOT/
# 第一次或许需要下载一些东西，可能比较慢
make BOARD=esp32-wroom-32 flash -C examples/hello-world/   
# 访问ESP32, 访问成功后，需要点击板子的左侧重启按钮。
make BOARD=esp32-wroom-32 term -C examples/hello-world/ 
```
出现以下类似内容即可：
> 2024-04-01 10:57:40,738 # main(): This is RIOT! (Version: 2024.04-devel-586-g48a8e6)
> 2024-04-01 10:57:40,740 # Hello World!
> 2024-04-01 10:57:40,744 # You are running RIOT on a(n) esp32-wroom-32 board.
> 2024-04-01 10:57:40,749 # This board features a(n) esp32 CPU.

#### 2.2.2 方法二: 容器工具链下载
##### (1) 容器及容器安装
首先什么是容器呢？可以参考GPT4的解释：
容器是一种轻量级的、可移植的、自足的软件封装技术，它能够在几乎任何环境中一致地运行应用程序。容器化应用程序包括应用程序本身以及其所有依赖项，但与传统虚拟化方法相比，容器共享主机系统的内核，因此更为高效。
下面是容器的主要特点：
1. 轻量级：容器使用主机操作系统，与其他容器共享其内核，而不需要自己的操作系统。这使得它们比虚拟机更轻，启动更快，占用的资源更少。
2. 可移植性：由于容器包含了应用程序及其所有依赖项，因此它们可以在任何支持容器技术的系统中运行，无论这些系统在哪里部署（本地服务器、公有云、私有云、个人电脑等）。
3. 一致性：容器提供了一致的运行环境，开发者可以在本地构建和测试容器，然后在任何地方部署，无需担心依赖问题。
4. 隔离性：每个容器都在自己的进程空间中运行，互不干扰。这提供了额外的安全层，并允许每个容器有自己的网络和文件系统接口。
5. 可扩展性和可复制性：容器可以快速和自动地扩展、部署和复制。
6. 微服务架构：容器适合微服务架构，因为它们允许开发者将复杂应用程序分解为一组独立的、可独立部署和扩展的小服务。
最著名的容器技术是 Docker，但还有其他许多选项，如 rkt、LXC 和 LXD 等。Kubernetes 是一种常用的容器编排工具，它可以自动化部署、扩展和管理容器化应用程序。

想进一步了解容器，可看[这篇文章](https://zhuanlan.zhihu.com/p/187505981)以及[官网文档](https://docs.docker.com/get-started/)。
容器安装有很多方法，[这篇文章](https://www.runoob.com/docker/ubuntu-docker-install.html)提供了几种比较可行的方法，这里提供最简单的安装方法，执行以下代码即可。

```bash
curl -fsSL https://test.docker.com -o test-docker.sh
sudo sh test-docker.sh                    # sudo 需要注意，可能输入密码

dockerd-rootless-setuptool.sh install

# 测试容器安装是否成功
sudo docker run --name hello hello-world

# 测试完，即可删除测试的容器和镜像
sudo docker rm -vf hello
sudo docker rmi hello-world
```

如果终端出现以下信息，容器即安装成功，可正常运行。

> Hello from Docker!
> This message shows that your installation appears to be working correctly.
> To generate this message, Docker took the following steps:
> The Docker client contacted the Docker daemon.
> The Docker daemon pulled the "hello-world" image from the Docker Hub.
> The Docker daemon created a new container from that image which runs the
> The Docker daemon streamed that output to the Docker client, which sent it
> To try something more ambitious, you can run an Ubuntu container with:
> $ docker run -it ubuntu bash
> Share images, automate workflows, and more with a free Docker ID:
> https://hub.docker.com/
> For more examples and ideas, visit:
> https://docs.docker.com/get-started/


##### (2) ESP32 容器工具链下载和编译方法

容器安装完成后，拉取官方提供的镜像, 并进行编译测试，这里使用最简单的hello-word的案例。具体如下：

```bash
sudo docker pull schorcht/riotbuild_esp32_espressif_gcc_8.4.0

# sudo chmod 777 /dev/ttyUSB*
cd ~/RIOT/
BUILD_IN_DOCKER=1 DOCKER="sudo docker" \
   DOCKER_IMAGE=schorcht/riotbuild_esp32_espressif_gcc_8.4.0 \
   make BOARD=esp32-wroom-32 flash -C examples/hello-world/
# 编译成功后，访问端口，并点击开发板左侧的重启按钮
make BOARD=esp32-wroom-32 term -C examples/hello-world/ 
```

终端会出现以下编译信息包含以下部分：
> ......
> Compressed 17184 bytes to 11719...
> Wrote 17184 bytes (11719 compressed) at 0x00001000 in 0.5 seconds (effective 279.6 kbit/s)...
> Hash of data verified.
> Compressed 3072 bytes to 85...
> Wrote 3072 bytes (85 compressed) at 0x00008000 in 0.1 seconds (effective 473.8 kbit/s)...
> Hash of data verified.
> Compressed 102096 bytes to 44068...
> Wrote 102096 bytes (44068 compressed) at 0x00010000 in 1.3 seconds (effective 616.2 kbit/s)...
> Hash of data verified.
> Leaving...
> Hard resetting via RTS pin...
> make: Leaving directory ...

访问端口并重启，会打印包括以下内容:
> ...
> 2024-04-01 11:04:00,497 # main(): This is RIOT! (Version: 2024.04-devel-480-gd76fc)
> 2024-04-01 11:04:00,500 # Hello World!
> 2024-04-01 11:04:00,503 # You are running RIOT on a(n) esp32-wroom-32 board.
> 2024-04-01 11:04:00,507 # This board features a(n) esp32 CPU.

官网提供比较详细的安装教程，如遇到问题，可先浏览[官方 RIOT Docker Toolchain文档](https://doc.riot-os.org/group__cpu__esp32.html#esp32_riot_docker_toolchain)。
至此，RIOT系统及编译环境已全部完成，需要记住对应方法的编译烧写方式，后续都要用到，可多试试"RIOT/examples/" 和 "RIOT/tests/"下的案例，尝试去修改这些案例，学会去用RIOT系统。
## 3 实验代码获取
