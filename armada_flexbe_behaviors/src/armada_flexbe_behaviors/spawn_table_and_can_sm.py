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
Created on Mon Jul 18 2022
@author: Dan
'''
class spawn_table_and_canSM(Behavior):
	'''
	spawning obj into sim
	'''


	def __init__(self):
		super(spawn_table_and_canSM, self).__init__()
		self.name = 'spawn_table_and_can'

		# parameters of this behavior
		self.add_parameter('table_path', '/home/csrobot/.gazebo/gazebo_models/table/model.sdf')
		self.add_parameter('table', 'table')
		self.add_parameter('namespace', 'test')
		self.add_parameter('frame', 'world')
		self.add_parameter('coke_path', '/home/csrobot/.gazebo/gazebo_models/coke_can/model.sdf')
		self.add_parameter('coke_can', 'coke_can')
		self.add_parameter('item_pose_x', 0)
		self.add_parameter('item_pose_y', -8)
		self.add_parameter('item_pose_z', 0)
		self.add_parameter('grab_item_pose_z', 2)
		self.add_parameter('theta', -45)
		self.add_parameter('side_wall_path', '/home/csrobot/building_editor_models/4_side_wall/model.sdf')
		self.add_parameter('side_wall', 'side_wall')
		self.add_parameter('side_wall_x', 0)
		self.add_parameter('side_wall_y', 0)
		self.add_parameter('wall_1_path', '/home/csrobot/building_editor_models/wall_1/model.sdf')
		self.add_parameter('wall_1_pose_x', 1)
		self.add_parameter('wall_1_pose_y', -2)
		self.add_parameter('wall_1', 'wall_1')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:902 y:42, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('table',
										spawnObjectState(model_name=self.table, object_file_path=self.table_path, robot_namespace=self.namespace, reference_frame=self.frame),
										transitions={'continue': 'coke_can', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:202 y:40
			OperatableStateMachine.add('coke_can',
										spawnObjectState(model_name=self.coke_can, object_file_path=self.coke_path, robot_namespace=self.namespace, reference_frame=self.frame),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
