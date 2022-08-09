#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_behaviors.graspable_update_scene_sm import graspable_update_sceneSM
from armada_flexbe_behaviors.spawn_objects_front_sm import spawn_objects_frontSM
from armada_flexbe_states.stow import stow
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Aug 01 2022
@author: Dan Choy
'''
class demo_frontSM(Behavior):
	'''
	pick and place demo
	'''


	def __init__(self):
		super(demo_frontSM, self).__init__()
		self.name = 'demo_front'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(graspable_update_sceneSM, 'graspable_update_scene')
		self.add_behavior(spawn_objects_frontSM, 'spawn_objects_front')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('spawn_objects_front',
										self.use_behavior(spawn_objects_frontSM, 'spawn_objects_front'),
										transitions={'finished': 'stow', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:292 y:31
			OperatableStateMachine.add('stow',
										stow(),
										transitions={'continue': 'graspable_update_scene', 'failed': 'stow'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:553 y:41
			OperatableStateMachine.add('graspable_update_scene',
										self.use_behavior(graspable_update_sceneSM, 'graspable_update_scene'),
										transitions={'finished': 'stow', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
