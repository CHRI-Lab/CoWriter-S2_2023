# setup-bluering

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

Open 3 terminals and execute the following instructions (in order and 1 per terminal):
- `make run-roscore`
- `make run-ui`
- `make run-cowriter`

Once you are inside the container, run `roslaunch bluering_letter_learning cowriter.launch   letter_model_dataset_directory:=/home/nao/catkin_ws/src/shape_learning/share/letter_model_datasets/bad_letters   format:=wave   audio_topic:=/audio/audio   nao_handedness:=left   openai_timeout:=20   openai_model:=gpt-3.5-turbo   openai_apikey:=$OPENAI_KEY   audio_outfile:="/home/nao/catkin_ws/test.wav"   log_audio_to_file:=false   shape_log:=/home/nao/catkin_ws/src/bluering_letter_learning/logs/23.05.24.1.log   openai_max_tokens:=25   audio_buflen:=20480   silent_threshold:=2500   min_silent_chunk_to_split:=100   nao_ip:=127.0.0.1`


## Comments



