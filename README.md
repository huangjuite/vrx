# MOST-Student-Project-2020

## Dependencies

- docker-ce
- nvidia-docker
- gpu support nvidia-docker

## Docker

pull docker image

```
$ docker pull juite/vrx
```

## Run docker

clone repo

```
$ git clone http://github.com/huangjuite/vrx
$ cd vrx
```

run docker

```
$ source docker_run.sh
```

open another docker terminal

```
$ source docker_join.sh
```

compile ros workspace in docker

```
# cd vrx
# source catkin_make.sh
```

open procman in docker

```
# cd vrx
# source environment.sh
# source start_vrx.sh
```

procman hot-key
| Hot Key | Function |
|-------------|--------------------|
| Ctrl + S | Start |
| Ctrl + T | Stop |
| Ctrl + R | Restart |
| Ctrl + A | Select All |

## Run experiments

plug a joystick first

### single boat RDPG

- start 02_gazebo_single
- start 03_teleop
- start 04_rl
- run evaluation

```
cd ~/vrx/catkin_ws/src/rl_navigation/src && python eval.py
```

### single boat HRVO

- start 02_gazebo_single
- start 03_teleop
- start 06_single_hrvo

### multi boat RDPG

- start 02_gazebo_multi
- start 04_rl_multi
- start 04_rl_state

### multi boat HRVO

- start 02_gazebo_multi
- start 06_multi_hrvo

### rviz

- start rviz

## Video

click gif to watch full video

<p align="center">
    <a href="https://www.youtube.com/watch?v=G8S00fxOxpc"> 
        <img src="https://j.gifs.com/NLR4Ov.gif" width="640">
    </a>
</p>

## Reference

vrx gazebo : https://bitbucket.org/osrf/vrx/src/default/
