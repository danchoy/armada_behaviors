#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.spawn_model_state import spawnObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 06 2022
@author: Dan
'''
class spawn_itemSM(Behavior):
	'''
	calling items to spawn in sim
	'''


	def __init__(self):
		super(spawn_itemSM, self).__init__()
		self.name = 'spawn_item'

		# parameters of this behavior
		self.add_parameter('Model_Name', 'Table')
		self.add_parameter('file_path', '/home/csrobot/.gazebo/gazebo_models/table/model.sdf')
		self.add_parameter('item_name', '')
		self.add_parameter('frame', 'world')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:531 y:162, x:511 y:267
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:154 y:170
			OperatableStateMachine.add('Spawn_Object',
										spawnObjectState(model_name=self.Model_Name, object_file_path=self.file_path, robot_namespace=self.item_name, reference_frame=self.frame),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
