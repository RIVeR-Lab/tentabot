# Instructions on using Northeaster's Research Cluster

* Last Update: 02/20/2022
* Author: Hongyu Li (LHY)
* Email : li.hongyu1 dot northeastern.edu

This is a brief tutorial on using Northeastern's Research Cluster, aka. Discovery, aka. HPC.

The entire process may be problematic, however, it is definitely rewarding and benefitial to go through all of these. I had descent experience with Docker, but it still took me three days to figure this out.

## Request an account

Follow the instructions on the [Research Cluster website](https://rc-docs.northeastern.edu/en/latest/first_steps/get_access.html)

* ask for PI permission
* fill out the [form](https://service.northeastern.edu/tech?id=sc_cat_item&sys_id=0ae24596db535fc075892f17d496199c)

## Regarding Storage
In general, each user has two spaces for storage. One is under /home/NUID/, the other is under /scratch/NUID/.
For example, ```/home/li.hongyu1``` and ```/scratch/li.hongyu1/```

The idea is that the home directory is a small, permanent storage, while the scratch directory is a HUUUUGE but temporary storage.

Detailed usage information is [here](https://rc-docs.northeastern.edu/en/latest/storage/discovery_storage.html)

## Build a docker image for your working project

It would be better if you know how to build a Singularity image, since you can only use Singularity on the HPC.

However, if you only know Docker, that's fine. Singularity image can be easily converted from a Docker image.

Docker image Example can be found at https://github.com/akmandor/tentabot/blob/hongyu/Dockerfile

There are several points you need to pay extra attention:
* Choose a workable base image. That's your starting point. For ROS projects, I personnaly recommend **osrf/ros:noetic-desktop-full**
* Put everything you need into it! The image is **read-only** in Singularity on HPC. So you won't be able to make any modification to the system.
* However, if you want to make any changes to the code inside your project, that's still **possible**. You can make **directory bindings or --bind option** in Singularity. Example is given later.
* You might want to make sure Gazebo GUI works in the image (Not only gzserver) due to several reasons. The major reason is that many sensor simulations won't work without a display output (in my case, realsense plugin). Therefore, pay attention to these lines in my Dockerfile
```
RUN apt install libqt5gui5 -y
RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so.5
RUN apt install python3-tk -y
```

## Convert the Docker image into Singularity image

There are many ways of doing so. The way I'm doing it is:

* Upload the locally built Docker image to the DockerHub
* On HPC, load Singularity module using command ``` module load singularity ```. This is a frequent command, so I put this into ``` ~/.bashrc ```
* Pull the Docker image from Dockerhub using [singularity pull](https://sylabs.io/guides/3.7/user-guide/cli/singularity_pull.html), and it would be automatically converted into Singularity image. For example: ``` singularity pull tentabot.sif docker://lhy0807/tentabot:0217 ``` where .sif file is the desired name of the Singularity image.
* **IMPORTANT INFO:**, before you pull the Singularity image, make sure you do the following, or you might encounter ```No space left on device``` error. Singularity would generate a lot of cache files under the ```/tmp``` folder which is relatively small, therefore, you are recommended to store all the cache in your scratch directory. You can do so by setting ```$TMPDIR``` and ```$SINGULARITY_CACHEDIR```.

For example,
```
TMPDIR=/scratch/li.hongyu1/singularity_tmp/ SINGULARITY_CACHEDIR=/scratch/li.hongyu1/singularity_cache/ singularity pull tentabot.sif docker://lhy0807/tentabot:0217
```

## Start Singularity image

To use Nvidia suite (CUDA, etc.), make sure you add ```--nv``` flag

For ```--bind``` option, the first directory is your **local directory**, and the second directory is the **directory inside your Singularity image**. Note that: if the directory exists in your singularity image, files will be overwrite by your local directory.
```
singularity shell --nv --network=host --bind /home/li.hongyu1/tentabot:/home/catkin_ws/src/tentabot-main --bind /home/li.hongyu1/openai_ros:/home/catkin_ws/src/openai_ros-main tentabot.sif
```

## Work as a normal computer
Start by sourcing your workspace. For example, in my Singularity image, my workspace is under ```/home/catkin_ws/```, then I will use
```
source /home/catkin_ws/devel/setup.bash
```

## To use Gazebo Desktop

Log into [Open OnDemand (OOD)](http://ood.discovery.neu.edu/).

Find [Xfce Desktop (Alpha)](https://ood.discovery.neu.edu/pun/sys/dashboard/batch_connect/sys/Desktop/session_contexts/new)
![OOD](images/ood1.png)

Start a session depends on your need.
![OOD](images/ood2.png)

Use like a normal Linux

## Wierd Issues observed

* I encounted abnormal behaviors when choosing different GPU. For example, there is no issue at all when using V100, but encounter immediate crash using P100, K40, and K80. I couldn't figure out what was going on.
* Sometimes, when I try to allocate a GPU node, and it is successfully allocated, there is no GPU inside the node at all. I will exit this node, wait for a while (~5 minutes), and allocate again.