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
from armada_flexbe_behaviors.head_move_sm import head_moveSM
from armada_flexbe_behaviors.point_cloud_state_sm import point_cloud_stateSM
from armada_flexbe_states.spawn_model_state import spawnObjectState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 27 2022
@author: Dan
'''
class spawn_objects_frontSM(Behavior):
	'''
	Spawn objects in-front of robot
	'''


	def __init__(self):
		super(spawn_objects_frontSM, self).__init__()
		self.name = 'spawn_objects_front'

		# parameters of this behavior
		self.add_parameter('table_path', '/home/csrobot/.gazebo/gazebo_models/table/model.sdf')
		self.add_parameter('table', 'table')
		self.add_parameter('namespace', 'test')
		self.add_parameter('frame', 'world')
		self.add_parameter('demo_cube', 'demo_cube')
		self.add_parameter('item_pose_x', 150)
		self.add_parameter('item_pose_y', 0)
		self.add_parameter('item_pose_z', 0)
		self.add_parameter('grab_item_pose_z', 200)
		self.add_parameter('theta', -45)
		self.add_parameter('grab_item_pose_x', 90)
		self.add_parameter('grab_item_pose_y', 0)
		self.add_parameter('coke_path', '/home/csrobot/.gazebo/gazebo_models/demo_cube/model.sdf')

		# references to used behaviors
		self.add_behavior(head_moveSM, 'head_move')
		self.add_behavior(point_cloud_stateSM, 'point_cloud_state')
		self.add_behavior(torso_actionSM, 'torso_action')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:794 y:374, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:38 y:29
			OperatableStateMachine.add('table',
										spawnObjectState(model_name=self.table, object_file_path=self.table_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.item_pose_x, pose_y=self.item_pose_y, pose_z=self.item_pose_z),
										transitions={'continue': 'soda_can', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:724 y:153
			OperatableStateMachine.add('point_cloud_state',
										self.use_behavior(point_cloud_stateSM, 'point_cloud_state'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:238 y:31
			OperatableStateMachine.add('soda_can',
										spawnObjectState(model_name=self.demo_cube, object_file_path=self.coke_path, robot_namespace=self.namespace, reference_frame=self.frame, pose_x=self.grab_item_pose_x, pose_y=self.grab_item_pose_y, pose_z=self.grab_item_pose_z),
										transitions={'continue': 'torso_action', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:417 y:37
			OperatableStateMachine.add('torso_action',
										self.use_behavior(torso_actionSM, 'torso_action'),
										transitions={'finished': 'head_move', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:660 y:38
			OperatableStateMachine.add('head_move',
										self.use_behavior(head_moveSM, 'head_move'),
										transitions={'finished': 'point_cloud_state', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
