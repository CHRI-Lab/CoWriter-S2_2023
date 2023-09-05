# setup-redback

## Install Docker

Download Docker on your Ubuntu (https://docs.docker.com/engine/install/ubuntu/).

When it is installed run

```
sudo groupadd docker
sudo usermod -aG docker $USER
```

(replace $USER with your ubuntu username)

## RUN

Start Choregraphe and connect the simulation of the robot.

All the commands should be ran at the level of the `Makefile` (root of the project).

Pull the docker image associated to RedBack: `make pull`.

When the pull is finished, open 3 terminals and execute the following instructions (in order and 1 per terminal):
- `make run-gui`
- `make run-test`
- `make run-audio-chat`. Once you are inside the container, run `ros2 launch launch/nao_writing_launch.py`.

## Comments

Best working project so far