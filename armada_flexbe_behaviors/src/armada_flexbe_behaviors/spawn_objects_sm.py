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
class spawn_objectsSM(Behavior):
	'''
	spawning obj into sim
	'''


	def __init__(self):
		super(spawn_objectsSM, self).__init__()
		self.name = 'spawn_objects'

		# parameters of this behavior
		self.add_parameter('table_path', '/home/csrobot/.gazebo/gazebo_models/table/model.sdf')
		self.add_parameter('table', 'table')
		self.add_parameter('namespace', 'test')
		self.add_parameter('frame', 'world')
		self.add_parameter('coke_path', '/home/csrobot/.gazebo/gazebo_models/demo_cube/model.sdf')
		self.add_parameter('demo_cube', 'demo_cube')
		self.add_parameter('item_pose_x', 0)
		self.add_parameter('item_pose_y', -800)
		self.add_parameter('item_pose_z', 0)
		self.add_parameter('grab_item_pose_z', 200)
		self.add_parameter('theta', -45)
		self.add_parameter('side_wall_path', '/home/csrobot/building_editor_models/4_side_wall/model.sdf')
		self.add_parameter('side_wall', 'side_wall')
		self.add_parameter('side_wall_x', 0)
		self.add_parameter('side_wall_y', 0)
		self.add_parameter('wall_1_path', '/home/csrobot/building_editor_models/wall_1/model.sdf')
		self.add_parameter('wall_1_pose_x', 100)
		self.add_parameter('wall_1_pose_y', -100)
		self.add_parameter('wall_1', 'wall_1')
		self.add_parameter('grab_item_pose_x', 0)
		self.add_parameter('grab_item_pose_y', -790)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:902 y:42, x:34 y:553
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:25
			OperatableStateMachine.add('side_wall',
										spawnObjectState(model_name=self.side_wall, object_file_path=self.side_wall_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.side_wall_x, pose_y=self.side_wall_y, pose_z=self.item_pose_z),
										transitions={'continue': 'wall_1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:366 y:23
			OperatableStateMachine.add('table',
										spawnObjectState(model_name=self.table, object_file_path=self.table_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.item_pose_x, pose_y=self.item_pose_y, pose_z=self.item_pose_z),
										transitions={'continue': 'coke_can', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:195 y:23
			OperatableStateMachine.add('wall_1',
										spawnObjectState(model_name=self.wall_1, object_file_path=self.wall_1_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.wall_1_pose_x, pose_y=self.wall_1_pose_y, pose_z=self.item_pose_z),
										transitions={'continue': 'table', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:535 y:24
			OperatableStateMachine.add('coke_can',
										spawnObjectState(model_name=self.demo_cube, object_file_path=self.coke_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.grab_item_pose_x, pose_y=self.grab_item_pose_y, pose_z=self.grab_item_pose_z),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
