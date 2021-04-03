# Docker guide

## run a docker image

```bash
sudo docker run -it image_id command
# or
sudo docker run -it repo_name:tag_name command
# example
sudo docker run -it w29054iu bash
sudo docker run -it test_pcl:2 bash

```

## build a dockerfile

```bash
# go to the folder where the dockerfile is saved
sudo docker build . -t image_name:tag_name
```

## save changes made in image

```bash
# when you are still inside the image
exit
# go back to your local terminal
# list all container id
sudo docker pc -a
# commit
sudo docker commit container_id image_name:tag_name
# example
sudo docker commit w2342uh pcl_test:3
```