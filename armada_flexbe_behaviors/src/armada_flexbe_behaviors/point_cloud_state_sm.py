#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.calc_xyz_service import PCL_CalculateXYZ
from armada_flexbe_states.concatenate_pointcloud_service_state import concatenatePointCloudState
from armada_flexbe_states.get_pointcloud_service_state import getPointCloudState
from armada_flexbe_states.pointcloud_passthrough_filter_service_state import pointCloudPassthroughFilterState
from armada_flexbe_states.publish_pointcloud_state import publishPointCloudState
from armada_flexbe_states.sac_segmentation_service_state import pointCloudSacSegmentationState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jul 18 2022
@author: Dan
'''
class point_cloud_stateSM(Behavior):
	'''
	getting pointcloud from headcamera and filter it
	'''


	def __init__(self):
		super(point_cloud_stateSM, self).__init__()
		self.name = 'point_cloud_state'

		# parameters of this behavior
		self.add_parameter('topic', '/head_camera/depth_registered/points')
		self.add_parameter('x_min', 0)
		self.add_parameter('y_min', -100)
		self.add_parameter('z_min', 100)
		self.add_parameter('x_max', 150)
		self.add_parameter('y_max', 100)
		self.add_parameter('z_max', 200)
		self.add_parameter('publish_topic', 'trans_cloud')
		self.add_parameter('single_point_topic', 'single_point')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1008 y:399, x:57 y:367
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pointcloud_list = []
		_state_machine.userdata.pointcloud_in = 0
		_state_machine.userdata.pointcloud_out = 0
		_state_machine.userdata.snapshot_pose_list = ['above','robot_left','robot_right']
		_state_machine.userdata.current_snapshot_step = 0
		_state_machine.userdata.target_pose = ['']
		_state_machine.userdata.combined_pointcloud = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('get_pc_state',
										getPointCloudState(camera_topic=self.topic),
										transitions={'continue': 'concate_pc', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list'})

			# x:255 y:39
			OperatableStateMachine.add('concate_pc',
										concatenatePointCloudState(),
										transitions={'continue': 'pc_passthrough_filter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud'})

			# x:475 y:37
			OperatableStateMachine.add('pc_passthrough_filter',
										pointCloudPassthroughFilterState(x_min=self.x_min, x_max=self.x_max, y_min=self.y_min, y_max=self.y_max, z_min=self.z_min, z_max=self.z_max),
										transitions={'continue': 'segmentation_state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'combined_pointcloud': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:480 y:379
			OperatableStateMachine.add('publish_pc_state',
										publishPointCloudState(topic=self.publish_topic),
										transitions={'continue': 'publish_single_point', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'combined_pointcloud'})

			# x:750 y:444
			OperatableStateMachine.add('publish_single_point',
										publishPointCloudState(topic=self.single_point_topic),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'pointcloud_out'})

			# x:478 y:181
			OperatableStateMachine.add('segmentation_state',
										pointCloudSacSegmentationState(),
										transitions={'continue': 'calculate_xyz', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'combined_pointcloud'})

			# x:706 y:248
			OperatableStateMachine.add('calculate_xyz',
										PCL_CalculateXYZ(),
										transitions={'continue': 'publish_pc_state', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'pointcloud_out'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
