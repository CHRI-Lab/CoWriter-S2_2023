#!/usr/bin/env python3
'''
This script is used to record & process data from the nao_ros_cowriter
experiment. It defines a ShapeCounter class, which listens to various 
ROS topics, counts and records various data related to the interaction 
between a user and a robot during the experiment.

The ShapeCounter class contains methods to handle the messages received
from different ROS topics, update counters & record data in CSV files.
The class keeps track of user-drawn shapes, robot-drawn shapes, words, 
corrections received and responded to, and corrections ignored.

In the `if __name__ == "__main__"` block, the script creates a 
ShapeCounter object, initializes ROS node, and sets up subscribers to 
the appropriate topics with callback methods from the ShapeCounter 
class. It listens to the following topics:
- /user_shapes: user-drawn shapes
- write_traj: robot-drawn shapes
- words_to_write: words requested by the user
- test_learning: test request
- stop_learning: stop request

The script uses argparse to parse command-line arguments for the names 
of the CSV files to store the data. It creates CSV writers for each 
file and passes them to the ShapeCounter object upon creation.

The rospy.spin() call is used to keep the script running, processing
callbacks until the ROS node is shut down. Once the node is shut down, 
the CSV files are automatically closed as they are managed by context
managers (created using `with` statement).
'''


import argparse
import csv
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from typing import List, Optional


class ShapeCounter:
    """
    A class to count and record various data related to the interaction
    between a user and a robot during a nao_ros_cowriter experiment.
    """

    def __init__(self, writer: csv.writer, writer_letters: csv.writer):  # type: ignore

        # Init csv writers
        self.writer = writer
        self.writer_letters = writer_letters

        # Init topics
        self.shapes_topic: str = "/user_shapes"
        self.letter_topic: str = 'write_traj'
        self.words_topic: str = 'words_to_write'
        self.test_topic: str = 'test_learning'
        self.stop_topic: str = 'stop_learning'

        # Init event info
        self.time_first_event: Optional[rospy.Time] = None
        self.user_shapes: List = []
        self.strokes: List = []
        self.number_of_user_shapes: int = 0
        self.number_of_strokes: int = 0
        self.number_of_corrections_received_this_word: int = 0
        self.interaction_finished: bool = False

        self.number_robot_shapes: int = 0
        # Don't count the first drawing as a 'correction'
        self.number_corrections_responded_to_this_word: int =- 1  

        self.number_of_words: int = 0
        self.number_of_corrections_ignored: int = 0
        self.previous_word: Optional[str] = None
        self.number_of_words_before_test: int = 0

    def check_time_first_event(self) -> None:
        """
        If time_first_event has not been set, set it
        to the current time.
        """
        if self.time_first_event is None:
            self.time_first_event = rospy.Time.now()

    def on_user_traj(self, message) -> None:
        """
        Callback function that is triggered when a new user trajectory
        is received. It processes the trajectory, updates the counters, 
        and writes the data to the appropriate CSV files.

        :param message: The user trajectory message.
        """
        # Update the time of the first event if it hasn't been set yet
        self.check_time_first_event()

        # If message has no poses and there are strokes, 
        # a shape has been completed
        if len(message.poses) == 0 and self.number_of_strokes > 0:
            self.number_of_user_shapes += 1
            self.number_of_strokes = 0
            self.strokes = []
            self.number_of_corrections_received_this_word += 1
            self.user_shapes.append(self.strokes)

        # If the message has poses and the interaction is not finished,
        # process the stroke
        elif len(message.poses) > 0 and not self.interaction_finished:
            stroke = []
            for point in message.poses:
                stroke.extend([point.pose.position.x, point.pose.position.y])

            # Convert stroke data to a string & write it to the CSV file
            stroke_string: str = ','.join([str(stroke0) for stroke0 in stroke])
            self.writer_letters.writerow(
                [str(self.number_of_user_shapes) + ',' +
                 str(self.number_of_strokes) + ',' + stroke_string])
            self.number_of_strokes += 1
            self.writer.writerow(
                ['Word', 'Number of corrections received',
                 'Number of corrections responded to',
                 'Number of corrections ignored'])

    def on_robot_traj(self, message):
        """
        Callback function that is triggered when a new robot trajectory
        is received. It updates the counters related to the robot's 
        actions.

        :param message: The robot trajectory message.
        """
        # Update the time of the first event if it hasn't been set yet
        self.check_time_first_event()

        # Increment the counters for robot shapes and corrections 
        # responded to
        self.number_robot_shapes += 1
        self.number_corrections_responded_to_this_word += 1

    def on_word(self, message):
        """
        Callback function that is triggered when a new word is received.
        It updates the counters and writes the data to the appropriate 
        CSV file.

        :param message: The word message.
        """
        self.check_time_first_event()

        # If the message is not 'end', update the word counter
        if message.data != 'end':
            self.number_of_words += 1

            # Calculate the number of corrections ignored for this word
            number_of_corrections_ignored_this_word = (
                self.number_of_corrections_received_this_word -
                self.number_corrections_responded_to_this_word)

            # Print information about the previous word
            print('Number of corrections received for previous word: ' +
                  f' {self.number_of_corrections_received_this_word}')
            print('Number of corrections responded to for previous word: ' +
                  f'{self.number_corrections_responded_to_this_word}')
            print('Number of corrections ignored: ' +
                  f'{number_of_corrections_ignored_this_word}')

            # Update the total number of corrections ignored and write 
            # data to the CSV file
            self.number_of_corrections_ignored += number_of_corrections_ignored_this_word
            self.writer.writerow([
                self.previous_word, 
                str(self.number_of_corrections_received_this_word),
                str(self.number_corrections_responded_to_this_word),
                str(number_of_corrections_ignored_this_word)])

            # Reset counters for the next word
            self.number_corrections_responded_to_this_word = -1
            self.number_of_corrections_received_this_word = 0

        # Message is 'end', write row
        else:  # Message is 'end', write
            self.writer.writerow(['Word', 'Number of corrections received',
                                  'Number of corrections responded to',
                                  'Number of corrections ignored'])

        # Print the number of word requests received
        print('-------------------Number of word requests received: ' +
              f'{self.number_of_words}  (\'+{message.data}+\')')
        self.previous_word = message.data

    def on_test_request_received(self, message) -> None:
        """
        Callback function that is triggered when a test request is 
        received. It updates the number_of_words_before_test counter.

        :param message: The test request message.
        """
        print(f'Number of words before test: {self.number_of_words}')
        self.number_of_words_before_test = self.number_of_words

    def on_stop_request_received(self, message):
        """
        Callback function that is triggered when a stop request is 
        received. It finalizes the interaction, writes the final 
        statistics to the CSV file, and prints the results.

        :param message: The stop request message.
        """
        # Record interaction as finished and send an end message
        self.interaction_finished = True
        end_message = String()
        end_message.data = 'end'
        self.on_word(end_message)

        # Print the final statistics
        print(
            f'Total number of user-drawn shapes: {self.number_of_user_shapes}')
        print('Total number of robot-drawn shape messages:' + 
              f'{self.number_robot_shapes}')
        print('Total number of corrections ignored: '
              f'{self.number_of_corrections_ignored}')
        print(f'Total number of words: {self.number_of_words}')

        # Calculate the duration of the interaction and print it
        time_stop = rospy.Time.now()
        duration = time_stop - self.time_first_event
        print(f'Total time: {duration.to_sec()}')

        # Write the final statistics to the CSV file
        self.writer.writerow(['Total number of words', 
                             str(self.number_of_words)])
        self.writer.writerow(['Number of words before test', 
                             str(self.number_of_words_before_test)])
        self.writer.writerow(['Total number of robot-drawn letters',
                             str(self.number_robot_shapes + 
                             2 * self.number_of_words)])
        self.writer.writerow(['Total number of user-drawn shapes', 
                             str(self.number_of_user_shapes)])
        self.writer.writerow(['Total number of user-drawn shapes responded to', 
                             str(self.number_of_user_shapes - \
                                 self.number_of_corrections_ignored)])
        self.writer.writerow(['Total number of corrections ignored', 
                             str(self.number_of_corrections_ignored)])


if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description='Record results of rosbag')
    parser.add_argument(
        'output', 
        action="store",
        help='a string containing the name of csv file to write to')
    parser.add_argument(
        'output_letters', 
        action="store",
        help='a string containing the name of csv file to write letter' +
             'strokes to')
    args = parser.parse_args()

    # Init writers for ShapeCounter
    # Use 'with' keyword to open with context manager,
    # so that files automatically closed when block ends
    with open(args.output, 'w') as csvfile, open(args.output_letters, 'w') as csvfile_letters:
        writer = csv.writer(csvfile, delimiter=' ',
                            quoting=csv.QUOTE_MINIMAL)

        writer_letters = csv.writer(csvfile_letters, delimiter=' ',
                                    quoting=csv.QUOTE_MINIMAL)

        # Create ShapeCounter with csv writers. init rospy node
        shape_counter = ShapeCounter(writer, writer_letters)
        rospy.init_node('shape_counter')

        # Init subscribers with topics and methods from ShapeCounter
        user_traj = rospy.Subscriber(
            shape_counter.shapes_topic, Path, shape_counter.on_user_traj)
        robot_traj = rospy.Subscriber(
            shape_counter.letter_topic, Path, shape_counter.on_robot_traj)
        words_subscriber = rospy.Subscriber(
            shape_counter.words_topic, String, shape_counter.on_word)
        test_subscriber = rospy.Subscriber(
            shape_counter.test_topic, 
            Empty, 
            shape_counter.on_test_request_received)
        stop_subscriber = rospy.Subscriber(
            shape_counter.stop_topic, 
            Empty, 
            shape_counter.on_stop_request_received)
        print('Waiting for rosbag to start')

        # Spin node until shutdown
        rospy.spin()
