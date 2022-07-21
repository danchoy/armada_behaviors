#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_Dan_flexbe_behaviors.torso_action_sm import torso_actionSM
from armada_flexbe_behaviors.spawn_objects_sm import spawn_objectsSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 20 2022
@author: Dan
'''
class pick_demoSM(Behavior):
	'''
	picking soda can demo
	'''


	def __init__(self):
		super(pick_demoSM, self).__init__()
		self.name = 'pick_demo'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(spawn_objectsSM, 'spawn_objects')
		self.add_behavior(torso_actionSM, 'torso_action')

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
			OperatableStateMachine.add('spawn_objects',
										self.use_behavior(spawn_objectsSM, 'spawn_objects'),
										transitions={'finished': 'torso_action', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:252 y:65
			OperatableStateMachine.add('torso_action',
										self.use_behavior(torso_actionSM, 'torso_action'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
