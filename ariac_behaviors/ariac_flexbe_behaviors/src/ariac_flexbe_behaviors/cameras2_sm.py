#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_support_flexbe_states.equal_state import EqualState
from ariac_support_flexbe_states.replace_state import ReplaceState
from ariac_flexbe_states.detect_part_camera_ariac_state import DetectPartCameraAriacState
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 18 2020
@author: Joost van Aken
'''
class Cameras2SM(Behavior):
	'''
	cameras
	'''


	def __init__(self):
		super(Cameras2SM, self).__init__()
		self.name = 'Cameras2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1166 y:283, x:970 y:277
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part_type'], output_keys=['arm', 'part_pose'])
		_state_machine.userdata.bin = 'part_type'
		_state_machine.userdata.bin1 = 'gear_part'
		_state_machine.userdata.bin2 = 'piston_rod'
		_state_machine.userdata.bin3 = ''
		_state_machine.userdata.bin4 = ''
		_state_machine.userdata.bin5 = 'pulley_part'
		_state_machine.userdata.bin6 = 'gastket_part'
		_state_machine.userdata.arm = ''
		_state_machine.userdata.arm1 = '/ariac/arm1'
		_state_machine.userdata.arm2 = '/ariac/arm2'
		_state_machine.userdata.ref_frame_bin2 = 'arm2_linear_arm_actuator'
		_state_machine.userdata.ref_frame_bin3 = 'arm1_linear_arm_actuator'
		_state_machine.userdata.ref_frame_bin4 = 'arm1_linear_arm_actuator'
		_state_machine.userdata.ref_frame_bin5 = 'arm1_linear_arm_actuator'
		_state_machine.userdata.ref_frame_bin6 = 'arm1_linear_arm_actuator'
		_state_machine.userdata.ref_frame_bin1 = 'arm2_linear_arm_actuator'
		_state_machine.userdata.camera_topic_bin1 = '/ariac/logical_camera_1'
		_state_machine.userdata.camera_topic_bin2 = '/ariac/logical_camera_2'
		_state_machine.userdata.camera_topic_bin3 = '/ariac/logical_camera_3'
		_state_machine.userdata.camera_topic_bin4 = '/ariac/logical_camera_4'
		_state_machine.userdata.camera_topic_bin5 = '/ariac/logical_camera_5'
		_state_machine.userdata.camera_topic_bin6 = '/ariac/logical_camera_6'
		_state_machine.userdata.camera_frame_bin1 = 'locical_camera_1_frame'
		_state_machine.userdata.camera_frame_bin2 = 'locical_camera_2_frame'
		_state_machine.userdata.camera_frame_bin3 = 'locical_camera_3_frame'
		_state_machine.userdata.camera_frame_bin4 = 'locical_camera_4_frame'
		_state_machine.userdata.camera_frame_bin5 = 'locical_camera_5_frame'
		_state_machine.userdata.camera_frame_bin6 = 'locical_camera_6_frame'
		_state_machine.userdata.part_pose = ''
		_state_machine.userdata.part_type = ''
		_state_machine.userdata.config_name_bin1 = 'bin1PreGrasp'
		_state_machine.userdata.config_name_bin2 = 'bin2PreGrasp'
		_state_machine.userdata.config_name_bin3 = 'bin3PreGrasp'
		_state_machine.userdata.config_name_bin4 = 'bin4PreGrasp'
		_state_machine.userdata.config_name_bin5 = 'bin5PreGrasp'
		_state_machine.userdata.config_name_bin6 = 'bin6PreGrasp'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.move_group = 'manipulator'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:490
			OperatableStateMachine.add('StateBin6',
										EqualState(),
										transitions={'true': 'Bin6', 'false': 'StateBin5'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin6'})

			# x:30 y:132
			OperatableStateMachine.add('StateBin2',
										EqualState(),
										transitions={'true': 'Bin2', 'false': 'StateBin1'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin2'})

			# x:30 y:224
			OperatableStateMachine.add('Statebin3',
										EqualState(),
										transitions={'true': 'Bin3', 'false': 'StateBin2'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin3'})

			# x:30 y:316
			OperatableStateMachine.add('StateBin4',
										EqualState(),
										transitions={'true': 'Bin4', 'false': 'Statebin3'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin4'})

			# x:30 y:408
			OperatableStateMachine.add('StateBin5',
										EqualState(),
										transitions={'true': 'Bin5', 'false': 'StateBin4'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin5'})

			# x:193 y:66
			OperatableStateMachine.add('Bin1',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart1'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm2'})

			# x:192 y:209
			OperatableStateMachine.add('Bin3',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart3'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm1'})

			# x:192 y:286
			OperatableStateMachine.add('Bin4',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart4'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm1'})

			# x:192 y:363
			OperatableStateMachine.add('Bin5',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart5'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm1'})

			# x:192 y:440
			OperatableStateMachine.add('Bin6',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart6'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm1'})

			# x:192 y:132
			OperatableStateMachine.add('Bin2',
										ReplaceState(),
										transitions={'done': 'DetectCameraPart2'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'arm', 'result': 'arm2'})

			# x:355 y:102
			OperatableStateMachine.add('DetectCameraPart1',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR2PreGrasp_Bin1', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin1', 'camera_topic': 'camera_topic_bin1', 'camera_frame': 'camera_frame_bin1', 'part': 'part_type', 'pose': 'part_pose'})

			# x:355 y:182
			OperatableStateMachine.add('DetectCameraPart2',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR2PreGrasp_Bin2', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin2', 'camera_topic': 'camera_topic_bin2', 'camera_frame': 'camera_frame_bin2', 'part': 'part_type', 'pose': 'part_pose'})

			# x:354 y:246
			OperatableStateMachine.add('DetectCameraPart3',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR1PreGrasp_Bin3', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin3', 'camera_topic': 'camera_topic_bin3', 'camera_frame': 'camera_frame_bin3', 'part': 'part_type', 'pose': 'part_pose'})

			# x:355 y:310
			OperatableStateMachine.add('DetectCameraPart4',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR1PreGrasp_Bin4', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin4', 'camera_topic': 'camera_topic_bin4', 'camera_frame': 'camera_frame_bin4', 'part': 'part_type', 'pose': 'part_pose'})

			# x:355 y:373
			OperatableStateMachine.add('DetectCameraPart5',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR1PreGrasp_Bin5', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin5', 'camera_topic': 'camera_topic_bin5', 'camera_frame': 'camera_frame_bin5', 'part': 'part_type', 'pose': 'part_pose'})

			# x:355 y:445
			OperatableStateMachine.add('DetectCameraPart6',
										DetectPartCameraAriacState(time_out=0.5),
										transitions={'continue': 'MoveR1PreGrasp_Bin6', 'failed': 'failed', 'not_found': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'not_found': Autonomy.Off},
										remapping={'ref_frame': 'ref_frame_bin6', 'camera_topic': 'camera_topic_bin6', 'camera_frame': 'camera_frame_bin6', 'part': 'part_type', 'pose': 'part_pose'})

			# x:520 y:459
			OperatableStateMachine.add('MoveR1PreGrasp_Bin6',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR1PreGrasp_Bin6', 'control_failed': 'MoveR1PreGrasp_Bin6', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin6', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:525 y:127
			OperatableStateMachine.add('MoveR2PreGrasp_Bin1',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR2PreGrasp_Bin1', 'control_failed': 'MoveR2PreGrasp_Bin1', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin1', 'move_group': 'move_group', 'move_group_prefix': 'arm2', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:525 y:196
			OperatableStateMachine.add('MoveR2PreGrasp_Bin2',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR2PreGrasp_Bin2', 'control_failed': 'MoveR2PreGrasp_Bin2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin2', 'move_group': 'move_group', 'move_group_prefix': 'arm2', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:526 y:259
			OperatableStateMachine.add('MoveR1PreGrasp_Bin3',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR1PreGrasp_Bin3', 'control_failed': 'MoveR1PreGrasp_Bin3', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin3', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:527 y:322
			OperatableStateMachine.add('MoveR1PreGrasp_Bin4',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR1PreGrasp_Bin4', 'control_failed': 'MoveR1PreGrasp_Bin4', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin4', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:524 y:390
			OperatableStateMachine.add('MoveR1PreGrasp_Bin5',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'MoveR1PreGrasp_Bin5', 'control_failed': 'MoveR1PreGrasp_Bin5', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin5', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:30 y:40
			OperatableStateMachine.add('StateBin1',
										EqualState(),
										transitions={'true': 'Bin1', 'false': 'failed'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'bin', 'value_b': 'bin1'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
