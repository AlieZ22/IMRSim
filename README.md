# IMRSim说明文档

## Overview

基于交错式磁记录（InterlacedMagnetic Recording，简写IMR）的机械硬盘是近年来具有前景的一种新型存储设备。以传统磁记录（Conventional Magnetic Recording，简写CMR）为基础的机械硬盘在容量增长方面已接近极限。虽然基于叠瓦式磁记录（Shingled Magnetic Recording，简写SMR）的机械硬盘能够一定程度提升数据存储密度，但是它在实际应用中面临着较大的问题。IMR是一种最近被提出的技术，相较于SMR具有更高的数据存储密度和更低的写入放大。IMR的发展进步能够推动存储设备的革新，而交错式磁盘的模拟是对IMR技术进行研究的重要手段，但是现在公开资料中并不存在一款可用的IMR磁盘模拟器。

我们开发了一种交错式磁盘模拟器（IMRSim）来模拟交错式磁记录技术。IMRSim包括两种类型的磁道：顶部磁道和底部磁道。IMRSim中顶部磁道和底部磁道呈交错式分布，每个底部磁道都与两个相邻的顶部磁道部分重叠。利用CMR技术，IMRSim在其基础上模拟顶部磁道、底部磁道以及数据存储更新的策略。将IMRSim模拟器作为块设备驱动程序在Linux内核中实现，并通过压力测试工具fio将其与CMR磁盘进行对比测试，以此评估使用IMRSim模拟器产生的性能开销。

## Design

IMRSim是一款公开的交错式磁盘模拟器，我们运用块设备驱动开发原理以及Device Mapper框架在Linux内核中进行设计与实现。Device Mapper是Linux内核映射块设备的一个通用技术框架，提供了一种从逻辑设备到物理设备的映射机制，具体介绍可参考（[Linux 内核中的 Device Mapper 机制 | Forest (thickforest.github.io)](https://thickforest.github.io/2014/10/39054332/)），该框架对于整体设计的理解至关重要，但这里不做过多的介绍。

### 整体设计

整体上将IMRSim设计成内核模块以及用户接口程序两个部分，如图所示：

![整体设计](.\images\IMR模拟器整体设计.jpg)

- 内核模块（IMRSim内核模块以及Device Mapper）。内核模块基于Device Mapper框架进行设计，用于构建磁盘模拟器的结构并实现所需的功能。
- 用户接口程序（IMRSim配置工具以及用户接口程序）。IMRSim利用Devcie Mapper提供的ioctl接口构建了一套可供用户使用的配置工具，用户接口程序利用C标准库在用户空间进行开发，利用提供的配置工具打通用户与磁盘模拟器交互的通道。

### 内核模块的设计

内核模块主要负责构建目标驱动插件（target driver）来对IMRSim的存储结构以及模拟器功能进行模拟。存储结构即解决磁盘模拟器文件中元数据以及用户数据如何放置的问题，而模拟器功能包括读写功能和接口功能。

对于Linux操作系统而言，一切皆文件。IMRSim模拟器文件的存储结构如下图所示：

![存储结构](.\images\IMR模拟器存储结构.png)

元数据包括磁盘统计信息以及zone状态信息，将元数据安排在磁盘模拟器文件的末尾有助于简化读写过程中的计算。

IMRSim对于读写功能的处理流程如下，其接口功能在用户接口程序的设计中一并介绍：

![读写流程](.\images\磁盘模拟器读写处理流程图.jpg)

### 用户接口程序的设计

用户空间主要负责根据用户的配置命令构建相应的映射关系，并构建逻辑设备。我们提供了一个用户接口程序来方便用户与最终构建的逻辑设备（IMRSim）进行交互。

用户接口程序基于内核模块暴露出的ioctl接口，所实现的功能可分为四大模块：

![功能模块](.\images\IMR模拟器功能模块.jpg)

具体功能如下表所示：

可以根据*code*的不同划分不同的功能类别，用单个字符表示，包括e（标识错误报告）、z（标识zone状态）、s（标识zone统计信息）以及l（标识配置相关）；*seq*代表某类功能的具体哪一种，用数字表示某类的第几个功能选项；而*arg*则是功能所需的一些参数（0个或1个）。

| 功能名称                   | code | seq  | 功能说明                  |
| -------------------------- | ---- | ---- | ------------------------- |
| show last   read error     | e    | 1    | 显示最近一次的读错误      |
| show last   write error    | e    | 2    | 显示最近一次的写错误      |
| enable   logging           | e    | 3    | 开启错误日志功能          |
| disable   logging          | e    | 4    | 关闭错误日志功能          |
| get   number of zones      | z    | 1    | 获得设备中有多少个zone    |
| get   default zone size    | z    | 2    | 获得一个zone的默认大小    |
| set   default zone size    | z    | 3    | 设置一个zone的默认大小    |
| reset   zone status        | z    | 4-5  | 重置某个zone的状态        |
| query   zone status        | z    | 6-8  | 查询zone的状态            |
| get all   zone stats       | s    | 1    | 获取所有zone的统计信息    |
| get zone   stats           | s    | 2    | 获取多个zone的统计信息    |
| get zone   stats by idx    | s    | 3    | 获取某个zone的统计信息    |
| reset all   zone stats     | s    | 4    | 重置所有zone统计信息      |
| reset   zone stats by lba  | s    | 5    | 重置lba所在zone的统计信息 |
| reset   zone stats by idx  | s    | 6    | 重置某个zone的统计信息    |
| set all   default config   | l    | 1    | 重置所有默认配置          |
| set zone   default config  | l    | 2    | 重置所有zone的配置        |
| reset dev   default config | l    | 3    | 重置设备配置              |
| get dev   config           | l    | 4    | 获取设备配置              |
| set read   penalty delay   | l    | 5    | 设置读延迟（微秒为单位）  |
| set write   penalty delay  | l    | 6    | 设置写延迟（微秒为单位）  |

## source

您可以在以下链接处获得IMRSim:

1. github:：
2. 百度网盘：
3. 与我们取得联系



## Building

系统要求：Linux Kernel < 3.16 （推荐版本：Ubuntu14.10）

1. 进入IMRSim所在目录，构建内核模块：

   ```bash
   $ make
   ```

2. 将驱动加载进内核：

   ```bash
   $ sudo make install
   $ sudo depmod --quick
   $ sudo modprobe dm-imrsim
   ```

3. 验证模块已成功加载并显示为可用的设备映射器目标：

   ```bash
   $ dmsetup targets
   ```

4. 准备一个块设备：

   （a）直接使用块设备（如，/dev/sdb）或分区（如，/dev/sdb1），容量要求大于256MB。

   （b）使用loop device。一个zone为256MB，可以自定义构建的zone数目，在下面的示例中构建了一个20GB的块设备（含有80个zone）：

   ```bash
   $ dd if=/dev/zero of=/tmp/imrsim1 bs=4096 seek=$(((256*80+2)*1024*1024/4096-1)) count=1
   $ losetup /dev/loop1 /tmp/imrsim1
   ```

5. 验证块设备的zone以及扇区数目：

   （a）zones：

   ```bash
   $ imrsim_util/imr_format.sh -z -d /dev/loop1
   ```

   （b）sectors：

   ```bash
   $ imrsim_util/imr_format.sh -d /dev/loop1
   ```

6. 创建IMRSim设备：

   ```bash
   $ echo "0 `imrsim_util/imr_format.sh -d /dev/loop1` imrsim /dev/loop1 0" | dmsetup create imrsim
   ```

   如果构建成功，将会创建IMRSim设备，存放于/dev/mapper/imrsim中。

7. 使用imrsim_util.c进行接口功能测试，或者使用fio等工具进行性能测试。



## Destory

在构建IMRSim之后，如需进行销毁，可以执行以下操作：

1. ```bash
   $ sudo dmsetup remove imrsim
   ```

2. ```bash
   $ losetup -d /dev/loop1
   ```

3. ```bash
   $ rmmod dm_imrsim
   ```

4. ```bash
   $ make clean
   ```



## How to use

### 功能测试

可以利用imrsim_util中提供的用户接口程序imrsim_util.c进行接口功能的测试，该程序接受的命令格式如下：

```bash
$ ./imrsim_util /dev/mapper/<dm_device_name> <code> <seq> <arg>
```

其中：

- <dm_device_name>为构建的逻辑设备名称，应该填入imrsim
- \<code>为功能类别的标识
- \<seq>为某类功能的具体种类标识
- \<arg>为可选参数

### 性能测试

以压力测试工具fio进行性能测试为例，测试指标为：IOPS、带宽以及延迟。

（1）对20GB的IMRSim模拟器进行顺序写测试：

```bash
$ fio --filename=/dev/mapper/imrsim --iodepth=128 --ioengine=libaio --direct=1 --rw=write --bs=4k --size=20g --numjobs=1 --runtime=3000 --group_reporting --name=test-write
```

（2）对20GB的IMRSim模拟器进行随机写测试：

```bash
$ fio --filename=/dev/mapper/imrsim --iodepth=128 --ioengine=libaio --direct=1 --rw=randwrite --bs=4k --size=20g --numjobs=1 --runtime=3000 --group_reporting --name=test-rand-write
```

*iodepth*是可以调整的，通过扩大*iodepth*的值可以提高硬盘利用率以获得性能的峰值。更多有关fio的测试方法可以参考（[IO性能测试工具使用_懂点代码的博客-CSDN博客_io测试](https://blog.csdn.net/qq_37674086/article/details/86636159)）

## Contact us

Author: Zeng Zhimin

Email: im_zzm@126.com







---------------------------------------------------------------------------------------------------------

## 修改
