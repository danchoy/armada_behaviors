#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from armada_flexbe_states.concatenate_pointcloud_service_state import concatenatePointCloudState as armada_flexbe_states__concatenatePointCloudState
from armada_flexbe_states.get_pointcloud_service_state import getPointCloudState as armada_flexbe_states__getPointCloudState
from armada_flexbe_states.pointcloud_passthrough_filter_service_state import pointCloudPassthroughFilterState as armada_flexbe_states__pointCloudPassthroughFilterState
from armada_flexbe_states.publish_pointcloud_state import publishPointCloudState as armada_flexbe_states__publishPointCloudState
from armada_flexbe_states.sac_segmentation_service_state import pointCloudSacSegmentationState as armada_flexbe_states__pointCloudSacSegmentationState
from armada_flexbe_states.snapshot_commander_state import snapshotCommanderState as armada_flexbe_states__snapshotCommanderState
from armada_flexbe_states.step_iterator_state import stepIteratorState
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
		self.add_parameter('y_min', -1)
		self.add_parameter('z_min', 0)
		self.add_parameter('x_max', 2)
		self.add_parameter('y_max', 1)
		self.add_parameter('z_max', 2)
		self.add_parameter('publish_topic', 'trans_cloud')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:890 y:319, x:130 y:365
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
			# x:37 y:42
			OperatableStateMachine.add('snapshotCommander',
										armada_flexbe_states__snapshotCommanderState(),
										transitions={'continue': 'concate_pc', 'take_snapshot': 'get_pc_state', 'failed': 'snapshot_iterator'},
										autonomy={'continue': Autonomy.Off, 'take_snapshot': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'snapshot_pose_list': 'snapshot_pose_list', 'current_snapshot_step': 'current_snapshot_step', 'target_pose': 'target_pose'})

			# x:236 y:175
			OperatableStateMachine.add('get_pc_state',
										armada_flexbe_states__getPointCloudState(camera_topic=self.topic),
										transitions={'continue': 'snapshot_iterator', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list'})

			# x:577 y:36
			OperatableStateMachine.add('passthrough_filter',
										armada_flexbe_states__pointCloudPassthroughFilterState(x_min=self.x_min, x_max=self.x_max, y_min=self.y_min, y_max=self.y_max, z_min=self.z_min, z_max=self.z_max),
										transitions={'continue': 'segmentation', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'combined_pointcloud', 'pointcloud_out': 'pointcloud_out'})

			# x:591 y:284
			OperatableStateMachine.add('publish_pc',
										armada_flexbe_states__publishPointCloudState(topic=self.publish_topic),
										transitions={'continue': 'finished', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud': 'pointcloud_out'})

			# x:585 y:164
			OperatableStateMachine.add('segmentation',
										armada_flexbe_states__pointCloudSacSegmentationState(),
										transitions={'continue': 'publish_pc', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_in': 'pointcloud_out', 'pointcloud_out': 'pointcloud_out'})

			# x:56 y:175
			OperatableStateMachine.add('snapshot_iterator',
										stepIteratorState(),
										transitions={'continue': 'snapshotCommander', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'iterator_in': 'current_snapshot_step', 'iterator_out': 'current_snapshot_step'})

			# x:338 y:33
			OperatableStateMachine.add('concate_pc',
										armada_flexbe_states__concatenatePointCloudState(),
										transitions={'continue': 'passthrough_filter', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pointcloud_list': 'pointcloud_list', 'combined_pointcloud': 'combined_pointcloud'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
