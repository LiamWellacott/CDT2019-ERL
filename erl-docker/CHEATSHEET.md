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

     docker rmi -f $(docker images --filter "dangling=true" -q --no-trunc)  
