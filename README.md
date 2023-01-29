## Abstract

The emerging interlaced magnetic recording (IMR) technology achieves a higher areal density for hard disk drive (HDD) over the conventional magnetic recording (CMR) technology. IMR-based HDD interlaces top tracks and bottom tracks, where each bottom track is overlapped with two neighboring top tracks. Thus, top tracks can be updated without restraint, whereas bottom tracks can be updated by the time-consuming read-modify-write (RMW) update strategy. Therefore, the layout of the tracks between the IMR-based HDD and the CMR-based HDD is much different. Unfortunately, there has been no related disk simulator and product available to the public, which motivates us to develop an open-source IMR disk simulator to provide a platform for further research.

We implement the first public IMR disk simulator, called IMRSim, as a block device driver in the Linux kernel, simulating the interlaced tracks and implementing many state-of-the-art data placement strategies. IMRSim is built on the actual CMR-based HDD to precisely simulate the I/O performance of IMR drives. While I/O operations in CMR-based HDD are easy to visualize, RMW strategy and multi-stage allocation strategy in IMR are inherently dynamic. Therefore, we further graphically demonstrate how IMRSim processes I/O requests in the visualization mode. We release IMRSim as an open-source IMR disk simulation tool and hope to attract more scholars into related research on IMR technology.



## Design

IMRSim is the first open-source simulator for IMR drives. We design and implement it in the Linux kernel using the principle of block device driver development and Device Mapper (DM) framework.

On the whole, we design IMRSim into two parts: the kernel module and the user interface program:

- **The kernel module** is designed based on the Device Mapper framework to build the structure of the disk simulator and implement the required functions (read, write, stage allocation strategy, etc.).
- **The user interface program** is a extensible tool to build the channel between the user and the simulator.  IMRSim uses the ioctl interface provided by Devcie Mapper to build a set of configuration tools that can be used by users. The user interface program is developed in the user space using the C standard library.

More details can be found in the paper (IMRSim: A Disk Simulator for Interlaced Magnetic Recording Technology).



## Source (version 1.1.0)

You can get IMRSim V1.1.0 at the following link:

1. github: [AlieZ22/IMRSim: A Disk Simulator for Interlaced Magnetic Recording Technology (github.com)](https://github.com/AlieZ22/IMRSim)
3. get in touch with us.



## Build

operating system: Linux（Recommended version: Ubuntu14.10）

1. Enter the directory where `IMRSim` is located and build the kernel module:

   ```bash
   $ make
   ```

2. Load the driver into the kernel:

   ```bash
   $ sudo make install
   $ sudo depmod --quick
   $ sudo modprobe dm-imrsim
   ```

4. prepare a block device：

   （a）Use block devices (eg, /dev/sdb) or partitions (eg, /dev/sdb1) directly, and the capacity requirement is greater than 256MB.

   （b）Use loop device. A zone is 256MB, and the number of zones can be customized. In the following example, a 20GB block device (containing 80 zones) is constructed:

   ```bash
   $ dd if=/dev/zero of=/tmp/imrsim1 bs=4096 seek=$(((256*80+2)*1024*1024/4096-1)) count=1
   $ losetup /dev/loop1 /tmp/imrsim1
   ```

5. Verify the zone and number of sectors of the block device：

   （a）zones：

   ```bash
   $ imrsim_util/imr_format.sh -z -d /dev/loop1
   ```

   （b）sectors：

   ```bash
   $ imrsim_util/imr_format.sh -d /dev/loop1
   ```

5. Create `IMRSim` device：

   The generic form:

   ```bash
   $ echo "0 <sectors> imrsim /dev/<your device> 0" | dmsetup create imrsim
   ```

   Take loop device as an example: 

   ```bash
   $ echo "0 `imrsim_util/imr_format.sh -d /dev/loop1` imrsim /dev/loop1 0" | dmsetup create imrsim
   ```

   If the build is successful, the IMRSim device will be created and stored in `/dev/mapper/imrsim`.

6. Use `imrsim_util.c` for interface function testing, or use tools such as `fio` for performance testing, or perform other tests in the `file system`.



## Destroy

After building IMRSim, to destroy it, you can do the following:

1. ```bash
   $ sudo dmsetup remove imrsim
   ```

2. ```bash
   $ sudo losetup -d /dev/loop1
   ```

3. ```bash
   $ sudo rmmod dm_imrsim
   ```

4. ```bash
   $ sudo make clean
   ```



## How to use

### Function Testing

The user interface program `imrsim_util.c` provide a tool, `imrsim_util`, can be used to test the interface function. The command format accepted by this program is as follows:

```bash
$ ./imrsim_util /dev/mapper/<dm_device_name> <code> <seq> <arg>
```

- <dm_device_name> is the built logical device name and should be filled in `imrsim`.
- \<code> identifies the functional category.
- \<seq> identifies the specific type of a certain type of function.
- \<arg> is an optional parameter.

Different functional categories can be divided according to *code*, which are represented by a single character, including `e` (identifying error report), `z` (identifying zone status), `s` (identifying zone statistics), and `l` (identifying configuration related). 

*seq* represents the specific type of a certain type of function, and uses numbers to indicate the number of function options of a certain type; and *arg* is some parameters (0 or 1) required by the function.

| function name            | code | seq  | function description                                      |
| ------------------------ | ---- | ---- | --------------------------------------------------------- |
| show last read error     | e    | 1    | Display the last read error                               |
| show last write error    | e    | 2    | Display the last write error                              |
| enable logging           | e    | 3    | Enable error log function                                 |
| disable logging          | e    | 4    | Turn off error logging                                    |
| get number of zones      | z    | 1    | Get how many zones are in the device                      |
| get default zone size    | z    | 2    | Get the default size of a zone                            |
| set default zone size    | z    | 3    | Set the default size of a zone                            |
| reset zone status        | z    | 4-5  | Reset the status of a zone                                |
| query zone status        | z    | 6-8  | Query the status of a zone                                |
| get all zone stats       | s    | 1    | Get statistics for all zones                              |
| get zone stats           | s    | 2    | Get statistics for multiple zones                         |
| get zone stats by idx    | s    | 3    | Get statistics for a certain zone                         |
| reset all zone stats     | s    | 4    | Reset all zone statistics                                 |
| reset zone stats by lba  | s    | 5    | Reset the statistics of the zone where the lba is located |
| reset zone stats by idx  | s    | 6    | Reset statistics for a zone                               |
| set all default config   | l    | 1    | Reset all default configurations                          |
| set zone default config  | l    | 2    | Reset the configuration of all zones                      |
| reset dev default config | l    | 3    | Reset device configuration                                |
| get dev config           | l    | 4    | Get device configuration                                  |
| set read penalty delay   | l    | 5    | Set read delay                                            |
| set write penalty delay  | l    | 6    | Set write delay                                           |

### Performance Testing

Taking the performance test of the stress test tool fio as an example, the test indicators are: IOPS, bandwidth and delay.

（1）4K sequential write test on 20GB IMRSim simulator:

```bash
$ fio --filename=/dev/mapper/imrsim --iodepth=128 --ioengine=libaio --direct=1 --rw=write --bs=4k --size=20g --numjobs=1 --runtime=3000 --group_reporting --name=test-write
```

（2）4K random write test on 20GB IMRSim simulator:

```bash
$ fio --filename=/dev/mapper/imrsim --iodepth=128 --ioengine=libaio --direct=1 --rw=randwrite --bs=4k --size=20g --numjobs=1 --runtime=3000 --group_reporting --name=test-rand-write
```

*iodepth* can be adjusted. By expanding the value of *iodepth*, the hard disk utilization can be increased to obtain the peak performance.



## Security

The project passed the security inspection of CNNVD and was included by the Open Source Security Community (OSCS).

[![Security Status](https://www.murphysec.com/platform3/v3/badge/1617938547948945408.svg?t=1)](https://www.murphysec.com/accept?code=8701786a3068b7a168c90d796e96a43c&type=1&from=2&t=2)



## Contact us

Author:  Zeng zhimin

Email: im_zzm@126.com


