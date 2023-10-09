#!/usr/bin/env python3

import os
import rospy
import rostest
import sys
import unittest
import logging
import tempfile
import numpy as np
from unittest.mock import MagicMock
from nav_msgs.msg import Path
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../nodes'))
from learning_words_nao import SubscriberCallbacks, StateManager, configure_logging
from nao_settings import NaoSettings, PhraseManagerGPT

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '../include'))
from wrapper_class import DeviceManager, PublisherManager
from state_machine import StateMachine

from letter_learning_interaction.msg import Shape as ShapeMsg
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Empty, Header

class TestLearningWordsNao(unittest.TestCase):
    def setUp(self):
        self.logger = logging.getLogger("test_logger")
        self.logger.propagate = False  # disable propagation to root logger
        
        self.nao_settings = NaoSettings()
        self.device_manager = DeviceManager()
        self.publish_manager = PublisherManager()
        self.state_machine = StateMachine()
        self.phrase_manager_gpt = PhraseManagerGPT('english')
        
        self.publish_manager.init_publishers()
        
        self.generated_word_logger = logging.getLogger("word_logger")
        self.generated_word_logger = configure_logging(self.generated_word_logger)

        self.subscriber_callbacks = SubscriberCallbacks(
            nao_settings=self.nao_settings,
            device_manager=self.device_manager,
            managerGPT=self.phrase_manager_gpt,
            publish_manager=self.publish_manager,
            state_machine=self.state_machine
        )
        
        self.state_manager = StateManager(
            nao_settings=self.nao_settings,
            device_manager=self.device_manager,
            publish_manager=self.publish_manager,
            subscriber_callbacks=self.subscriber_callbacks,
            generated_word_logger=self.generated_word_logger
        )

    def tearDown(self):
        for handler in self.logger.handlers:
            self.logger.removeHandler(handler)

    # logging function
    def test_configure_logging_default_path(self):
        logger = configure_logging(self.logger)
        self.assertEqual(len(logger.handlers), 1)
        self.assertIsInstance(logger.handlers[0], logging.FileHandler)
        self.assertEqual(logger.level, logging.DEBUG)

    def test_configure_logging_valid_path(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "test.log")
            logger = configure_logging(self.logger, path=path)
            self.assertEqual(len(logger.handlers), 1)
            self.assertIsInstance(logger.handlers[0], logging.FileHandler)
            self.assertEqual(logger.handlers[0].baseFilename, path)
            self.assertEqual(logger.handlers[0].level, logging.DEBUG)
            self.assertEqual(logger.handlers[0].formatter._fmt,
                             '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            self.assertEqual(logger.level, logging.DEBUG)

    def test_configure_logging_invalid_path(self):
        logger = configure_logging(self.logger, path=None)
        self.assertEqual(len(logger.handlers), 1)
        self.assertIsInstance(logger.handlers[0], logging.NullHandler)
        self.assertEqual(logger.level, logging.DEBUG)
        
    # SubscriberCallbacks class
    def test_on_user_drawn_shape_received(self):
        # setup test data
        shape_msg = ShapeMsg()
        shape_msg.path = [1,2,3,4,5,6]
        shape_msg.shape_type = 'a'
        subscriber_callbacks = self.subscriber_callbacks
        subscriber_callbacks.active_letter = 'a'
        subscriber_callbacks.state_machine.set_start('WAITING_FOR_FEEDBACK')
        
        # call method under test
        subscriber_callbacks.on_user_drawn_shape_received(shape_msg)

        # assert results
        self.assertEqual(len(subscriber_callbacks.demo_shapes_received), 1)
        self.assertEqual(subscriber_callbacks.demo_shapes_received[0].path, shape_msg.path)
        self.assertEqual(subscriber_callbacks.demo_shapes_received[0].shape_type, 'a')

    def test_on_shape_finished(self):
        subscriber_callbacks = self.subscriber_callbacks

        # call method under test
        subscriber_callbacks.on_shape_finished(None)

        # assert results
        self.assertTrue(subscriber_callbacks.shape_finished)

    def test_on_new_child_received(self):
        # setup test data
        nao_settings = self.subscriber_callbacks.nao_settings
        nao_settings.nao_writing = True
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.motion_proxy = MagicMock()
        nao_settings.posture_proxy = MagicMock()
        subscriber_callbacks = self.subscriber_callbacks
        message = 'test message'

        # call method under test
        subscriber_callbacks.on_new_child_received(message)

        # assert results
        self.assertEqual(nao_settings.motion_proxy.rest.call_count, 1)
        self.assertEqual(nao_settings.motion_proxy.setStiffnesses.call_count, 2)
        self.assertEqual(nao_settings.posture_proxy.goToPosture.call_count, 0)
        
    def test_on_word_received_string(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks
        subscriber_callbacks.state_machine.set_start('WAITING_FOR_FEEDBACK')
        message = 'test message'

        # call method under test
        subscriber_callbacks.on_word_received(message)

        # assert results
        self.assertEqual(subscriber_callbacks.word_received, message)
        
    def test_on_word_received_empty(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks
        subscriber_callbacks.state_machine.set_start('WAITING_FOR_LETTER_TO_FINISH')
        message = 'test message'

        # call method under test
        subscriber_callbacks.on_word_received(message)

        # assert results
        self.assertEqual(subscriber_callbacks.word_received, None)
        
    def test_on_clear_screen_received_service_unavailable(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks

        # call method under test
        subscriber_callbacks.on_clear_screen_received()

        # assert results
        self.assertRaises(rospy.ServiceException)

    def test_on_test_request_received(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks

        # call method under test
        subscriber_callbacks.on_test_request_received(Empty)

        # assert results
        self.assertTrue(subscriber_callbacks.test_request_received)

    def test_on_stop_request_received(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks

        # call method under test
        subscriber_callbacks.on_stop_request_received(Empty)

        # assert results
        self.assertTrue(subscriber_callbacks.stop_request_received)
        
    def test_on_set_active_shape_gesture(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks
        
        pt_header = Header(stamp=rospy.Time.now(), frame_id="odom")
        pt = Point(0.1, 0.2, 0.3)
        pt_stamp = PointStamped(header=pt_header, point=pt)

        # call method under test
        subscriber_callbacks.on_set_active_shape_gesture(pt_stamp)

        # assert results
        # BROKEN
        self.assertEqual(subscriber_callbacks.active_letter, '')

    def test_on_feedback_received_asking(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks
        nao_settings = subscriber_callbacks.nao_settings
        nao_settings.nao_writing = True
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.motion_proxy = MagicMock()
        nao_settings.posture_proxy = MagicMock()
        subscriber_callbacks.state_machine.set_start('ASKING_FOR_FEEDBACK')
        message = MagicMock()
        message.data = 'test message'
        
        # call method under test
        subscriber_callbacks.on_feedback_received(message)

        # assert results
        self.assertIsInstance(subscriber_callbacks.feedback_received, str)

    def test_on_feedback_received_responding(self):
        # setup test data
        subscriber_callbacks = self.subscriber_callbacks
        nao_settings = subscriber_callbacks.nao_settings
        nao_settings.nao_writing = True
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.motion_proxy = MagicMock()
        nao_settings.posture_proxy = MagicMock()
        subscriber_callbacks.state_machine.set_start('RESPONDING_TO_FEEDBACK')
        message = MagicMock()
        message.data = 'test message'
        
        # call method under test
        subscriber_callbacks.on_feedback_received(message)

        # assert results
        self.assertIsInstance(subscriber_callbacks.feedback_received, str)
        
    # StateManager class
    def test_handle_word_received_false(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = self.state_manager.subscriber_callbacks
        subscriber_callbacks.state_machine.set_start('WAITING_FOR_FEEDBACK')
        next_state = ''
        info_for_next_state = {'word_received': 'a'}

        # call method under test
        next_state_out, info_for_next_state_out = state_manager.handle_word_received(next_state, info_for_next_state)
        
        # assert results
        self.assertEqual(next_state_out, next_state)
        self.assertEqual(info_for_next_state_out, info_for_next_state)

        
    def test_handle_word_received_true(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = self.state_manager.subscriber_callbacks
        subscriber_callbacks.state_machine.set_start('WAITING_FOR_FEEDBACK')
        message = 'test message'
        subscriber_callbacks.on_word_received(message)
        next_state = ''
        exp_next_state_out = 'RESPONDING_TO_NEW_WORD'
        info_for_next_state = {'word_received': 'a'}

        # call method under test
        next_state_out, info_for_next_state_out = state_manager.handle_word_received(next_state, info_for_next_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, info_for_next_state)
    
    def test_check_stop_request_received_false(self):
        # setup test data
        state_manager = self.state_manager
        next_state = ''

        # call method under test
        next_state_out = state_manager.check_stop_request_received(next_state)
        
        # assert results
        self.assertEqual(next_state_out, next_state)

    def test_check_stop_request_received_true(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = self.state_manager.subscriber_callbacks
        subscriber_callbacks.on_stop_request_received(Empty)
        exp_next_state_out = 'STOPPING'
        next_state = ''
        
        # call method under test
        next_state_out = state_manager.check_stop_request_received(next_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
    
    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def test_respond_to_demonstration(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'demo_shapes_received': ''}
    #     exp_next_state_out = 'WAITING_FOR_ROBOT_TO_CONNECT'
    #     exp_info_for_next_state = ['state_came_from', 'RESPONDING_TO_DEMONSTRATION']
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.respond_to_demonstration(info_from_prev_state)

    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
    #     self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
        
    def test_respond_to_demonstration_with_full_word(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'demo_shapes_received': ''}
        exp_next_state_out = 'PUBLISHING_WORD'
        exp_info_for_next_state = ['state_came_from', 
                                   'RESPONDING_TO_DEMONSTRATION_FULL_WORD']
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.respond_to_demonstration_with_full_word(info_from_prev_state)

        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
        
    def test_publish_word(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'RESPONDING_TO_DEMONSTRATION',
                                'item_written': 'a',
                                'word_to_write': 'a',
                                'state_go_to': 'PUBLISHING_WORD'}
        exp_next_state_out = 'WAITING_FOR_LETTER_TO_FINISH'
        exp_info_for_next_state = ['state_came_from', 
                                   'PUBLISHING_WORD', 
                                   'state_go_to', 
                                   'PUBLISHING_WORD', 
                                   'item_written', 
                                   'a']
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.publish_word(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
        self.assertIn(exp_info_for_next_state[2], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[3], info_for_next_state_out.values())
        self.assertIn(exp_info_for_next_state[4], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[5], info_for_next_state_out.values())
        
    def test_wait_for_shape_to_finish(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'WAITING_FOR_LETTER_TO_FINISH',
                                'item_written':'a',
                                'state_go_to': 'PUBLISH_WORD'}
        exp_next_state_out = 'WAITING_FOR_LETTER_TO_FINISH'
        exp_info_for_next_state = ['state_came_from', 
                                   'WAITING_FOR_LETTER_TO_FINISH']
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_shape_to_finish(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
    
    def test_wait_for_shape_to_finish_stop_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'WAITING_FOR_LETTER_TO_FINISH'}
        exp_next_state_out = 'STOPPING'
        exp_info_for_next_state = ['state_came_from', 
                                   'WAITING_FOR_LETTER_TO_FINISH']
        subscriber_callbacks.on_stop_request_received(Empty)
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_shape_to_finish(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
        self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
    
    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def test_respond_to_test_card(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {}
    #     exp_next_state_out = 'WAITING_FOR_WORD'
    #     exp_info_for_next_state = ['state_came_from', 'RESPONDING_TO_TEST_CARD']
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = \
    #       state_manager.respond_to_test_card(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertIn(exp_info_for_next_state[0], info_for_next_state_out.keys())
    #     self.assertIn(exp_info_for_next_state[1], info_for_next_state_out.values())
        
    def test_stop_interaction(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {}
        exp_next_state_out = 'EXIT'
        exp_info_for_next_state = 0
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.stop_interaction(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_start_interaction(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {}
        exp_next_state_out = 'WAITING_FOR_WORD'
        exp_info_for_next_state = {'state_came_from': 'STARTING_INTERACTION'}
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.start_interaction(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_start_interaction_stop_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {}
        exp_next_state_out = 'STOPPING'
        exp_info_for_next_state = {'state_came_from': 'STARTING_INTERACTION'}
        subscriber_callbacks.on_stop_request_received(Empty)

        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.start_interaction(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_wait_for_word_wait(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'WAITING_FOR_WORD'}
        exp_next_state_out = 'WAITING_FOR_WORD'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_WORD'}
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_word(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    def test_wait_for_word_starting(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'STARTING_INTERACTION'}
        exp_next_state_out = 'WAITING_FOR_WORD'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_WORD'}
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_word(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    def test_wait_for_word_stop_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'STARTING_INTERACTION'}
        exp_next_state_out = 'STOPPING'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_WORD'}
        subscriber_callbacks.on_stop_request_received(Empty)

        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_word(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    def test_wait_for_feedback(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'STARTING_INTERACTION'}
        exp_next_state_out = 'WAITING_FOR_FEEDBACK'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_FEEDBACK'}

        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_wait_for_feedback_test_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        exp_next_state_out = 'WAITING_FOR_FEEDBACK'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_FEEDBACK'}
        subscriber_callbacks.on_test_request_received(Empty)

        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    def test_wait_for_feedback_stop_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        exp_next_state_out = 'STOPPING'
        exp_info_for_next_state = {'state_came_from': 'WAITING_FOR_FEEDBACK'}
        subscriber_callbacks.on_stop_request_received(Empty)

        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.wait_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_ask_for_feedback_publish_word(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'demo_shapes_received': '',
                                'state_came_from': 'PUBLISHING_WORD',
                                'item_written': 'a'}
        exp_next_state_out = 'WAITING_FOR_FEEDBACK'
        exp_info_for_next_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        
        # call method under test
        next_state_out, info_for_next_state_out = state_manager.ask_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def test_ask_for_feedback_publish_letter(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'demo_shapes_received': '',
    #                             'state_came_from': 'PUBLISHING_LETTER',
    #                             'shape_published': 'a'}
    #     exp_next_state_out = 'WAITING_FOR_FEEDBACK'
    #     exp_info_for_next_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.ask_for_feedback(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    def test_ask_for_feedback_test_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'demo_shapes_received': '',
                                'state_came_from': 'PUBLISHING_WORD',
                                'item_written': 'a'}
        exp_next_state_out = 'WAITING_FOR_FEEDBACK'
        exp_info_for_next_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        subscriber_callbacks.on_test_request_received(Empty)
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.ask_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_ask_for_feedback_stop_request(self):
        # setup test data
        state_manager = self.state_manager
        subscriber_callbacks = state_manager.subscriber_callbacks
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        info_from_prev_state = {'demo_shapes_received': '',
                                'state_came_from': 'PUBLISHING_WORD',
                                'item_written': 'a'}
        exp_next_state_out = 'STOPPING'
        exp_info_for_next_state = {'state_came_from': 'ASKING_FOR_FEEDBACK'}
        subscriber_callbacks.on_stop_request_received(Empty)
        
        # call method under test
        next_state_out, info_for_next_state_out = \
            state_manager.ask_for_feedback(info_from_prev_state)
        
        # assert results
        self.assertEqual(next_state_out, exp_next_state_out)
        self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    # def test_wait_for_robot_to_connect(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     subscriber_callbacks = state_manager.subscriber_callbacks
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'state_came_from': 'PUBLISHING_WORD',
    #                             'state_go_to': ['WAITING_FOR_ROBOT_TO_CONNECT']}
    #     exp_next_state_out = 'STOPPING'
    #     exp_info_for_next_state = {'state_came_from': 'PUBLISHING_WORD', 'state_go_to': []}
    #     subscriber_callbacks.on_stop_request_received(Empty)
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.wait_for_robot_to_connect(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    # def test_wait_for_robot_to_connect_stop_request(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     subscriber_callbacks = state_manager.subscriber_callbacks
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'state_came_from': 'PUBLISHING_WORD',
    #                             'state_go_to': ['WAITING_FOR_ROBOT_TO_CONNECT']}
    #     exp_next_state_out = 'WAITING_FOR_ROBOT_TO_CONNECT'
    #     exp_info_for_next_state = {'state_came_from': 'PUBLISHING_WORD', 'state_go_to': []}
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.wait_for_robot_to_connect(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    # def test_wait_for_tablet_to_connect(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'state_came_from': 'PUBLISHING_WORD',
    #                             'state_go_to': ['WAITING_FOR_TABLET_TO_CONNECT']}
    #     exp_next_state_out = 'WAITING_FOR_TABLET_TO_CONNECT'
    #     exp_info_for_next_state = {'state_came_from': 'PUBLISHING_WORD', 'state_go_to': []}
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.wait_for_tablet_to_connect(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
    
    # def test_wait_for_tablet_to_connect_stop_request(self):
    #     # setup test data
    #     state_manager = self.state_manager
    #     subscriber_callbacks = state_manager.subscriber_callbacks
    #     nao_settings = state_manager.nao_settings
    #     nao_settings.nao_writing = False
    #     nao_settings.nao_connected = False
    #     nao_settings.nao_standing = False
    #     nao_settings.nao_speaking = False
    #     info_from_prev_state = {'state_came_from': 'PUBLISHING_WORD',
    #                             'state_go_to': ['WAITING_FOR_TABLET_TO_CONNECT']}
    #     exp_next_state_out = 'STOPPING'
    #     exp_info_for_next_state = {'state_came_from': 'PUBLISHING_WORD', 'state_go_to': []}
    #     subscriber_callbacks.on_stop_request_received(Empty)
        
    #     # call method under test
    #     next_state_out, info_for_next_state_out = state_manager.wait_for_tablet_to_connect(info_from_prev_state)
        
    #     # assert results
    #     self.assertEqual(next_state_out, exp_next_state_out)
    #     self.assertEqual(info_for_next_state_out, exp_info_for_next_state)
        
    def test_get_next_phrase(self):
        # setup test data
        state_manager = self.state_manager
        nao_settings = state_manager.nao_settings
        nao_settings.nao_writing = False
        nao_settings.nao_connected = False
        nao_settings.nao_standing = False
        nao_settings.nao_speaking = False
        phrases = ['hi', 'bye']
        counter = 0
        
        # call method under test
        to_say, counter_out = state_manager.get_next_phrase(phrases, counter)
        
        # assert results
        self.assertEqual(to_say, phrases[0])
        self.assertEqual(counter_out, counter+1)
    
    
    def test_downsample_shape(self):
        # Create a simple example shape that is easier to check
        shape = np.array([0, 0, 0.5, 0.5, 1, 0, 1, 1, 0.5, 1, 0, 1])
        
        downsampled_shape = self.state_manager.downsample_shape(shape)
        
        # Test 1: Check downsampled_shape is twice length of NUMPOINTS 
        self.assertEqual(len(downsampled_shape), 
            2 * self.state_manager.NUMPOINTS_SHAPEMODELER)
    
        # Test 2: Check that the downsampled_shape is a numpy array
        self.assertTrue(isinstance(downsampled_shape, np.ndarray))
        
        # Test 3: Check that the output shape is a 2D array
        self.assertEqual(len(downsampled_shape.shape), 2)
        
        # Test 4: Check that the downsampled_shape has only one column
        self.assertEqual(downsampled_shape.shape[1], 1)
        
    def test_make_bounding_box_msg(self):
        # Test case 1: Default values
        bbox = [0.0, 0.0, 1.0, 1.0]
        msg = self.state_manager.make_bounding_box_msg(bbox)
        self.assertEqual(len(msg.data), 4)
        self.assertEqual(msg.data, bbox)
        self.assertEqual(msg.layout.dim[0].label, "bb")

        # Test case 2: Selected bbox
        bbox = [0.0, 0.0, 2.0, 2.0]
        msg = self.state_manager.make_bounding_box_msg(bbox, selected=True)
        self.assertEqual(len(msg.data), 4)
        self.assertEqual(msg.data, bbox)
        self.assertEqual(msg.layout.dim[0].label, "select")
        
    def test_make_traj_msg(self):
        FRAME1 = rospy.get_param('~writing_surface_frame_id', 'writing_surface')
        shaped_word = self.device_manager.text_shaper.shape_word(
            self.device_manager.word_manager)
        placed_word = self.device_manager.screen_manager.place_word(
            shaped_word)
        shaped_word = deepcopy(placed_word)
        # Test case 1: Default values
        shaped_word.letter_paths = [[[0,0], [0,1], [1,1], [1,0]]]
        delta_t = 0.1
        traj = self.state_manager.make_traj_msg(shaped_word, delta_t)

        self.assertIsInstance(traj, Path)
        self.assertEqual(traj.header.frame_id, FRAME1)

        expected_time = 0
        for idx, point in enumerate(traj.poses):
            self.assertIsInstance(point, PoseStamped)
            self.assertEqual(point.header.frame_id, FRAME1)
            self.assertEqual(point.header.seq, idx + 1)
            self.assertAlmostEqual(point.header.stamp.to_sec(), 
                                   expected_time, places=1)
            self.assertAlmostEqual(point.pose.position.x, 
                                   shaped_word.letter_paths[0][idx][0], places=2)
            self.assertAlmostEqual(point.pose.position.y, 
                                   shaped_word.letter_paths[0][idx][1], places=2)

            expected_time += delta_t

    # Commented method/function out because not presently in use
    # TODO: reintegrate or remove
    #def test_respond_to_new_word(self):
        # Set up initial state
        #info_from_prev_state = {'word_received': 'test'}
        
        # Call function under test
        #self.nao_settings.set_nao_interaction()
        #next_state, info_for_next_state = self.state_manager.respond_to_new_word(info_from_prev_state)
        
        # Check expected results
        #self.assertEqual(next_state, 'PUBLISHING_WORD')
        #self.assertEqual(info_for_next_state['state_came_from'], 'RESPONDING_TO_NEW_WORD')
        #self.assertEqual(info_for_next_state['word_to_write'], 'test')
        #self.assertIsInstance(info_for_next_state['shapes_to_publish'], list)
        #self.assertEqual(len(info_for_next_state['shapes_to_publish']), 4)  # Assuming 4 letters in "test"
        
        # Check that screen was cleared and publisher cleared an empty message
        #self.assertTrue(self.device_manager.screen_manager.clear_called)
        #self.assertTrue(self.publish_manager.pub_clear_called)
        
        # Check that phrase was spoken
        #self.assertTrue(self.nao_settings.nao_speak_and_log_phrase_called)
        
if __name__ == '__main__':
    rospy.init_node("learning_words_nao")
    rostest.rosrun('letter_learning_interaction', 
                   'test_learning_words_nao', 
                   TestLearningWordsNao, 
                   sys.argv)
