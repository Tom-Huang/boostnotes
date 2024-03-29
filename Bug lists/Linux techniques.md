# Linux techniques

## Switch between Nvidia and Intel Graphic cards

1. Open terminal, type `prime-select query` to see what graphic card you are using now.
2. Type `prime-select nvidia` or `prime-select intel`.

## Reinstall grub after moving /boot directory

1. Use USB to launch Ubuntu
2. Use the following command to see which partition belongs to /boot, remember the device name e.g. `/dev/nvme0n1p10`
```bash
sudo fdisk -l # show all partition
blkid # show UUID of all partition
```

3. Type the following to mount /boot partition:

```bash
sudo mkdir /mnt/ubuntu
sudo mkdir /mnt/ubuntu/boot
sudo mount /dev/nvme0n1p10 /mnt/ubuntu/boot
```

4. Type the following:

```bash
grub-install --boot-directory=/mnt/ubuntu/boot /dev/nvme0n1 # no p10 at the end
```

5. If `can not embed` `GTP no boot` similar error occurs, create a partition of 1 MB with no format and tagged with boot_grup(tag after creation in GParted). Then try step 4 again. If succeed, reboot. 

## The position for storing each partitions' UUID

/etc/fstab

## How to modify swap partition's UUID after moving the position

After changing the partition position in GParted, the file /etc/fstab didn't change the new position of swap space. This will lead to very slow boot time. To solve this problem, just do the following steps:

1. mkswap create swap space at the partition you specify, here is /dev/nvme0n1p9
```bash
sudo mkswap /dev/nvme0n1p9
```
2. Use this command to show the UUID of each partition, markdown the swap partition\'s UUID, e.g. /dev/nvme0n1p9
```bash
sudo blkid 
```
3. Open `/etc/fstab` file and change the UUID for swap partition with the one we markdown in step 2.
```bash
sudo vim /etc/fstab
```
4. run this to update the write
```bash
sudo update-initramfs -u
```

## How to modify the used version after installing different versions of same app
For example, if I install more than one version of clang
```bash
sudo update-alternatives --config clang
```

## Conda create and delete environments

```bash
conda create -n myenv python=3.6
conda activate myenv
conda remove -n myenv --all # remove all packages and the environment
```

## Git remove remote object

```bash
git rm file1.txt
git commit -m "remove file1.txt"
```

## OpenCV error `error: ‘CV_LOAD_IMAGE_GRAYSCALE’ was not declared in this scope`

modify the code which contains `CV_LOAD_IMAGE_GRAYSCALE` to `cv::IMREAD_GRAYSCALE`

`CV_LOAD_IMAGE_COLOR`->`cv::IMREAD_COLOR`
`CV_LOAD_IMAGE_UNCHANGED`->`cv::IMREAD_UNCHANGED`

## Make header file also can include some libraries in CMake project

when you add_executable in CMakeLists.txt, also add the header file as source.

## ROS subscriber callback function in a class

```c++
sub = n.subscribe("/camera/depth_registered/points", 1000, &Example::callBack, this);
```

## Open terminator with a certain layout and command run

[bash - How to open terminal, split to 9 terminals and switch between them using one script? - Unix & Linux Stack Exchange](https://unix.stackexchange.com/questions/168436/how-to-open-terminal-split-to-9-terminals-and-switch-between-them-using-one-scr/168445#168445)

## uninstall CUDA

```bash
cd /usr/local/cuda-10.0/bin/
sudo ./uninstall-cuda*
```

## delete all files except several files(filename1 and filename2 here)

```bash
rm -v !("filename1"|"filename2") 
```

## cmake downloading some file failed

it is because you install anaconda which also contains curl (for downloading). you should remove the curl in conda

```bash
conda remove curl
sudo apt-get install curl
# go to cmake source code folder
./bootstrap --system-curl
make
sudo make install
```

## ROS ERROR: Could not find a package configuration file provided by "nav_msgs" with any of the following names:

```bash
sudo apt-get install ros-melodic-nav-msgs
```

## can not find xx.h even if cmake find package and link correctly
go to the source cmakelists.txt of xx.h and see if there is a flag name BUILD_SHARED_LIBRARY and set it to TRUE

## ROS tf::lookupTransform(...) Error: passed to lookupTransform argument target_frame does not exist.

```cpp
// add one more wait line
tf_listener.waitForTransform("/CABIN", "/map", ros::Time(0), ros::Duration(0.001));
tf_listener.lookupTransform("/CABIN", "/map", ros::Time(0), transform_lidar_to_world);
```

## use PCL removeNANfromPointCloud doesn't work

if the input_pc is hand crafted, remember to set `input_pc.is_dense = false;` before removing NAN.

## locate a library in linux

```bash
ldconfig -p | grep liblz4
```

## when building pcl_catkin error: Undefined reference to LZ4_decompress_fast, LZ4_compressHC

```bash
# first make sure you install libflann-deve
sudo apt-get install libflann-dev
# go to /home/hcg/hcg/semester_project/heap_ws/build/pcl_catkin/pcl_src-prefix/src/pcl_src-build/kdtree/CMakeFiles
cd /home/hcg/hcg/semester_project/heap_ws/build/pcl_catkin/pcl_src-prefix/src/pcl_src-build/kdtree/CMakeFiles

# add -llz4 in the line in link.txt
```

or maybe move liblz4* inside anaconda/lib folder into a folder so that the system will not find it in conda

or it could be that your libraries in /usr/local/lib override the libraries with the same name in /usr/lib/x86_64-linux-gnu.
use the following command to find what the dynamic linked libraries is located:

```bash
ldd -d -r /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
```
the output might be:
```text
	linux-vdso.so.1 (0x00007ffce4ce8000)
	libpcl_common.so.1.8 => /usr/lib/x86_64-linux-gnu/libpcl_common.so.1.8 (0x00007fe2c3a76000)
	libflann_cpp.so.1.9 => /usr/local/lib/libflann_cpp.so.1.9 (0x00007fe2c3874000)
	libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007fe2c34a0000)
	libgomp.so.1 => /usr/lib/x86_64-linux-gnu/libgomp.so.1 (0x00007fe2c3261000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007fe2c3049000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007fe2c2c58000)
	libboost_system.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1 (0x00007fe2c2a53000)
	libboost_thread.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.65.1 (0x00007fe2c282e000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007fe2c260f000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fe2c2271000)
	/lib64/ld-linux-x86-64.so.2 (0x00007fe2c406d000)
	libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007fe2c206d000)
	librt.so.1 => /lib/x86_64-linux-gnu/librt.so.1 (0x00007fe2c1e65000)
undefined symbol: LZ4_decompress_safe_continue	(/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so)
undefined symbol: LZ4_decompress_safe	(/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so)
undefined symbol: LZ4_compress_HC_continue	(/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so)
undefined symbol: LZ4_resetStreamHC	(/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so)
undefined symbol: LZ4_setStreamDecode	(/usr/lib/x86_64-linux-gnu/libpcl_kdtree.so)
```
if there are undefined symbols, try to find the .so file located in /usr/local/lib:
```bash
libflann_cpp.so.1.9 => /usr/local/lib/libflann_cpp.so.1.9 (0x00007fe2c3874000)
```

these libraries located in /usr/local/lib might be installed from `sudo make install`. Move them away to a folder. Use ldd to check whether there are still undefined symbols.

```bash
ldd -d -r /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
```
the output is:
```text
	linux-vdso.so.1 (0x00007fff8ddca000)
	libpcl_common.so.1.8 => /usr/lib/x86_64-linux-gnu/libpcl_common.so.1.8 (0x00007f52c249b000)
	libflann_cpp.so.1.9 => /usr/lib/x86_64-linux-gnu/libflann_cpp.so.1.9 (0x00007f52c228d000)
	libstdc++.so.6 => /usr/lib/x86_64-linux-gnu/libstdc++.so.6 (0x00007f52c1eb9000)
	libgomp.so.1 => /usr/lib/x86_64-linux-gnu/libgomp.so.1 (0x00007f52c1c7a000)
	libgcc_s.so.1 => /lib/x86_64-linux-gnu/libgcc_s.so.1 (0x00007f52c1a62000)
	libc.so.6 => /lib/x86_64-linux-gnu/libc.so.6 (0x00007f52c1671000)
	libboost_system.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1 (0x00007f52c146c000)
	libboost_thread.so.1.65.1 => /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.65.1 (0x00007f52c1247000)
	libpthread.so.0 => /lib/x86_64-linux-gnu/libpthread.so.0 (0x00007f52c1028000)
	libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007f52c0c8a000)
	/lib64/ld-linux-x86-64.so.2 (0x00007f52c2a92000)
	libdl.so.2 => /lib/x86_64-linux-gnu/libdl.so.2 (0x00007f52c0a86000)
	librt.so.1 => /lib/x86_64-linux-gnu/librt.so.1 (0x00007f52c087e000)
```
we can see that the library is looking for libflann.so.1.9 in /usr/lib/x86_64-linux-gnu
```text
libflann_cpp.so.1.9 => /usr/lib/x86_64-linux-gnu/libflann_cpp.so.1.9 (0x00007f52c228d000)
```

## clean old nvidia driver and cuda

```bash
sudo apt-get --purge -y remove 'cuda*'
sudo apt-get --purge -y remove 'nvidia*'
sudo reboot
```

## W: GPG error: http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY C8B3A55A6F3EFCDE

fix it with this link's information: [apt - How do I fix the GPG error "NO_PUBKEY"? - Ask Ubuntu](https://askubuntu.com/questions/13065/how-do-i-fix-the-gpg-error-no-pubkey)

## list all files beginning with something

```bash
ls -d abc*
```

## tmux page up faster

ctrl+b and then press \[, then use page up to scroll up faster

## pcl has no library visualization with pcl_catkin `fatal error: pcl/visualization/pcl_visualizer.h: No such file or directory`

make sure correct version of vtk can be found. pcl_catkin-1.8.0 corresponds to vtk-7.1.1. vtk-9.0.0 doesn't work. So we can first uninstall current vtk and then reinstall vtk-7.1.1.
```bash
# go to the build folder of existing vtk
sudo make uninstall
# install vtk-7.1.1, download the .zip file from the website and extract it
cd vtk-7.1.1
mkdir build && cd build && cmake .. && make 
sudo make install
```

## ImportError: dynamic module does not define module export function (PyInit__tf2)

```bash
>>> import tf2_py
Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/home/hcg/hcg/semester_project/heap_ws/devel/lib/python2.7/dist-packages/tf2_py/__init__.py", line 34, in <module>
    exec(__fh.read())
  File "<string>", line 38, in <module>
ImportError: dynamic module does not define module export function (PyInit__tf2)
```

## undefined reference to `typeinfo for YAML::BadConversion'

1. remove the preinstalled libyaml-cpp-dev with 
    ```bash
    sudo apt-get remove libyaml-cpp-dev
    ```
2. build a newer version of yaml-cpp from source >=0.6.2
    ```bash
    git clone https://github.com/jbeder/yaml-cpp.git
    cd yaml-cpp && mkdir build && cd build
    cmake .. && make
    sudo make install
    ```
## Specify path for cmake to find package

It is useful when you don't have sudo permission to install package to /usr/local. So you can build from source and let cmake find package here.

the path is the directory where you put the XXConfig.cmake file. It usually lies in the build folder where you run `cmake ..`.

```CMake
find_package(PCL 1.7.2 REQUIRED PATHS ~/hcg/software/pcl/pcl-pcl-1.7.2/build NO_DEFAULT_PATH)
```

## Where does `sudo apt install` put libraries and where does `sudo make install` put libraries in?

`sudo apt install` might put libraries in `/usr/lib`

`sudo make install` might put libraries in `/usr/local/lib`

## locate a file with a certain name

```bash
locate *filename*
```

## check which libraries an executable linked to

can be applied to executable and .so file

```
ldd -d -r /usr/local/lib/libflann_cpp.so.1.9
```

## when you use grep, don't use `*`

`grep *lz4*` will only search for `*lz4*` with `*` in the name. Just use `grep lz4` to search anything with `lz4` as part of the name

## clean a package with apt and all its dependencies

```bash
sudo apt-get autoremove --purge libpcl-dev
```

## rviz can't open .rviz config file

```bash
sudo apt-get dist-upgrade
```
also, if in your workspace you install your own version of yaml-cpp, you will have this problem. Just delete the yaml-cpp you install on your own will work