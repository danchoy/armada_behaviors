#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_behaviors.robot_move_sm import robot_moveSM
from armada_flexbe_behaviors.spawn_objects_sm import spawn_objectsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 18 2022
@author: Dan
'''
class spawn_moveSM(Behavior):
	'''
	spawning objects and moving robot
	'''


	def __init__(self):
		super(spawn_moveSM, self).__init__()
		self.name = 'spawn_move'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(robot_moveSM, 'robot_move')
		self.add_behavior(spawn_objectsSM, 'spawn_objects')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:911 y:31, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('spawn_objects',
										self.use_behavior(spawn_objectsSM, 'spawn_objects'),
										transitions={'finished': 'robot_move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:249 y:32
			OperatableStateMachine.add('robot_move',
										self.use_behavior(robot_moveSM, 'robot_move'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
