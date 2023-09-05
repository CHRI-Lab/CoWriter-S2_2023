# setup-boxjelly

## Install Docker

Download Docker on your Ubuntu (https://docs.docker.com/engine/install/ubuntu/).

When it is installed run

```
sudo groupadd docker
sudo usermod -aG docker $USER
```

(replace $USER with your ubuntu username)

## Run

Start Choregraphe and connect the simulation of the robot.

All the commands should be ran at the level of the `Makefile` (root of the project).

Pull the docker image associated to RedBack: `make pull`.

When the pull is finished, open 3 terminals and execute the following instructions (in order and 1 per terminal):
- `make run-robot`. Once you are inside the container, run `roslaunch letter_learning_interaction nao_learning.launch letter_model_dataset_directory:=/home/nao/NAOHW-Boxjelly/share/letter_model_datasets/alexis_set_for_children`.
- `make run-manager`
- `make run-child`

## Comments

Kind of works, the robot is drawing a letter, that's all

