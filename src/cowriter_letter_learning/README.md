# The CoWriter project: Teaching a robot to write

A set of ROS nodes which facilitate the user interaction allowing a robot to be taught handwriting.

Tested with Python 3.8.10 and ROS Noetic on Ubuntu 20.04 (LTS).

![Photo of CoWriter interaction](https://github.com/chili-epfl/cowriter_letter_learning/raw/master/doc/cowriter_demo.jpg)
## Usage
#### With a Choregraphe simulated Nao running or a actual Nao robot
(It should be noted that the actual Nao robot's IP address could be different from the default IP of the program.)
```
roslaunch letter_learning_interaction nao_learning.launch letter_model_dataset_directory:=/home/<username>/Desktop/<workspace>/share/letter_model_datasets/<model datasets>  #such as alexis_set_for_children

```
*(Alternatively, `rostopic pub /words_to_write std_msgs/String "use" -1` may be used to send words to write (e.g. 'use') manually, without detecting cards.)*
*Note that console output can be viewed with `rosrun rqt_console rqt_console`.*

