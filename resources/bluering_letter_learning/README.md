# bluering_letter_learning

This package contains a set of nodes useful for creating a user interaction around the premise of a Nao learning handwriting.

Tested with ROS noetic and python 3.8.

## Provided nodes:
- `interactive_learning.py`: the main node for managing the CoWriter interaction. Controls the robot to facilitate the interaction (the only node that initiates a session to the robot and controls the robot); manages the learning algorithm state; sends requests for shapes to be written and responds to received user feedback on said shapes. (requires the [shape_learning library](../shape_learning-pypkg), and a running `display_manager_server` node)

- `diagram_manager.py`: listens to audio input from pc's microphone; processes audio buffer; convert speech to text; generate commands / actions from the converted text and sends an event to `interactive_learning` to control the robot.

- `display_manager_server.py`: provides services which allow for access to a `ShapeDisplayManager`'s methods by other ROS nodes. I.e., allows multiple nodes to position shapes on the display (as needed by a shape learning algorithm) and request which shapes are present at a particular location (as needed to process user feedback on shapes), etc.

- `tablet_input_interpreter.py`: listens for tablet inputs from the user and translates them into shape-specific events based on the location at which they occurred. [requires a running `display_manager_server` node]

- `word_card_detector.py`: listens for frames which represent fiducial markers for a dictionary of words, and publishes the associated words (used to request a word to be written by the user). Tested with [chilitags for ROS](https://github.com/chili-epfl/ros_markers).


### diagram_manager
`diagram_manager` uses [audio_common/audio_capture](https://github.com/ros-drivers/audio_common/tree/master/audio_capture) to capture audio from pc's microphone. Audio is captured in chunk, each chunk is sent via ROS topic `/audio/audio` (can be customised by modifying the roslaunch argument)  

`diagram_manager` then receives and processes these audio chunks. Each audio chunk contains 320 bytes (note that we need to convert from default representation of audio data (8-bit) to 16-bit representation).  

`diagram_manager` has a variable `listening`, if this is set to `false`, audio chunk will not be processed. (Note that audio is always captured if microphone is on, it's just not processed)  
Variable `listening` will be set to `true` if topic `listening_signal` has event (topic name can be customised by setting parameter when running the program via `roslaunch`). Topic `listening_signal` receives a `String`, one of two value: 
- `word`: the robot is expecting only ONE word from human to write the word.
- `convo`: the robot is listening to human to maintain a conversation.

When in `convo` mode, `diagram_manager` will detect when human finishes talking by detecting a long continous pause. It detects pause using a `StaticSilenceDetector`, which implements a simple static thresholding. The threshold for silence is passed as an argument `silent_threshold`, chunks of which mean is < this `silent_threshold` value will be marked as silence chunk.  

For each audio chunk received, if it's not silence chunk, it will be appended to an audio buffer.  
After detecting a long silence, it checks if the current audio buffer is big enough (>= `audio_buflen`, which is an argument). If it's big enough, this audio buffer will be passed to a Speech-to-Text module to convert to text, and the audio buffer is reset to empty at the same time.  

`diagram_manager` also recognises command from voice input. Now it supports 3 types of commands, for each command, the "signature" texts are:
```js
COMMANDS = {
    'change_word': ['change word'], 
    'login': ['old user', 'old child'], 
    'register': ['new user', 'new child'],
}
```
For example, to command the robot to add a new child, one can use voice command: "Help me add a **new child** named Joey", or "Let's add a **new user** named Joey"  
To change word from voice command, put the word right after the command signature, for example: "Let's **change word** cat" (word to write: cat), "I want to **change word** dog" (word to write: dog) *(not tested yet)*


## Sample command
```
GOOGLE_APPLICATION_CREDENTIALS=/home/ubuntu/catkin_ws/flowing-elf-385414-757ca2faa900.json
roslaunch bluering_letter_learning cowriter.launch
   letter_model_dataset_directory:=/home/ubuntu/catkin_ws/shape_learning-pypkg/share/letter_model_datasets/bad_letters
   format:=wave
   audio_topic:=/audio/audio
   nao_handedness:=left
   openai_timeout:=20
   openai_model:=gpt-3.5-turbo
   openai_apikey:=sk-...
   openai_max_tokens:=100
   audio_outfile:="/home/ubuntu/catkin_ws/test.wav"
   log_audio_to_file:=false
   shape_log:=/home/ubuntu/catkin_ws/src/bluering_letter_learning/logs/23.05.24.1.log
   audio_buflen:=20480
   silent_threshold:=2500
   min_silent_chunk_to_split:=100
   nao_ip:=127.0.0.1
```
- `GOOGLE_APPLICATION_CREDENTIALS=/path/to/google/cloud/credentials`: credential file to use google cloud speech-to-text service.
- `letter_model_dataset_directory`: path to bad letters dataset. 
Must points to a directory containing files (or symlink to files) named `[a-zA-Z].dat`, one per letter.  
(note that these files are loaded 'on-demand', so if you know you won't be using a certain range of letter,
you do not need the corresponding datasets).  
You can learn more about these datasets (format, where to get them, how to create them...) in
the [`shape_learning` project](../shape_learning-pypkg).

- `shape_log`: path to store logs of trajectory generated.  
- `nao_ip`: robot IP
- `log_audio_to_file`: `true` if you want to also log the captured audio to file.
- `audio_outfile`: if `log_audio_to_file` is `true`, then captured audio will be logged to this path.
- `shape_log`: log the shapes collected in this session.
- `audio_buflen`: when in `convo` mode, size of the audio buffer must >= this value to be passed to a Speech-to-Text module.
- `silent_threshold`: audio chunks whose mean value is < this value are considered silence chunks.
- `min_silent_chunk_to_split`: number of continous silence chunks to indicate a finished sentence. ie., when `diagram_manager` detects `min_silent_chunk_to_split` continous silence chunks, it identifies this as the person finished speaking, and process the audio buffer.