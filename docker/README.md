# Install

To install the docker image with cuda support you need:

[Docker](https://www.docker.com/) and [Nvidia-Docker 2](https://github.com/NVIDIA/nvidia-docker). You need cuda compatible GPU (from Nvidia), make sure you follow the install procedure for both carefully, [this guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) should provide all of the required steps

# Build

To build the image run

```
make gaan_base
```

If needed CPU support only can be added!

# Run

If you are running for the first time do the following: 

```
xhost +si:localuser:root
XAUTH = /tmp/.docker.xauth-n
touch $XAUTH
xauth nlist :0 | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod a+r $XAUTH

./dev-first.bash
```

Once this is run once you can run the image with  ```./dev.bash``` found in the docker folder.
This will launch the image and mount your catkin workspace from your file system to /gaan-ws

To have additional terminals inside the docker environment you can re run ```./dev.bash``` from the new terminal.

When you run the container for the first time you will have to run ```catkin_make```, this builds the gaan packages which are unavailable to docker until the checkout of the repo is mounted in ```./dev-first.bash```.

# Add dependencies

To add new dependencies just instert the terminal command in the right dockerfile.

 - Anything related to Nvidia/cuda/cudnn etc in the ```gaan/nvidia/*``` package.
 - Anything related to ros in ```gaan/melodic/```
 - Anything related to tiago in ```gaan/tiago/```
 - Anything related to our solution in ```.```

# NOTES:

 - Not tested on anything else than ubuntu. Need to look at how to forward Xserver equivalent in macos and Windows, or use other tools to launch the image.

 - The camera/webcam device is located in /dev/video0 in ubuntu, need to edit dev.bash to give it the correct camera name. 

## Cheatsheet

Commit to changes on a docker image, should be used to save last compilation but the code should be stored on disk by correctly mounting the file system.

To Commit:

    1. Apply all the changes on the image. Write the code, Compile etc.

    2.
        ```
        sudo docker ps -a
        ```
        Will print the docker images running. Check for the CONTAINER ID
    3. Run
        ```
        sudo docker commit [CONTAINER ID] [new_image_name]
        ```
    4. You can check the new image with
        ```
        sudo docker images
        ```

    5. After that just run image with new name.
       ```
        docker run [new_image_name]
       ```



To remove docker images:
    ```
     docker rmi -f $(docker images --filter "dangling=true" -q --no-trunc)  
    ```

To have GUI run in host :
    ```
    xhost +si:local
    ```

Cmake for dlib worked when adding

if (NOT CUDA_CUBLAS_LIBRARIES)
   set(CUDA_CUBLAS_LIBRARIES /usr/local/cuda/lib64/libcublas.so)
endif()
