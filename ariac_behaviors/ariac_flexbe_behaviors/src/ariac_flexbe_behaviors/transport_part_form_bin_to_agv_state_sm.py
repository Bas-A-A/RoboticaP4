#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from ariac_flexbe_states.srdf_state_to_moveit_ariac_state import SrdfStateToMoveitAriac
from flexbe_states.wait_state import WaitState
from ariac_flexbe_states.compute_grasp_ariac_state import ComputeGraspAriacState
from ariac_flexbe_states.moveit_to_joints_dyn_ariac_state import MoveitToJointsDynAriacState
from ariac_flexbe_states.get_object_pose import GetObjectPoseState
from ariac_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from ariac_flexbe_behaviors.cameras2_sm import Cameras2SM
from ariac_support_flexbe_states.equal_state import EqualState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 12 2020
@author: Bas Abrahamse
'''
class transport_part_form_bin_to_agv_stateSM(Behavior):
	'''
	Test
	'''


	def __init__(self):
		super(transport_part_form_bin_to_agv_stateSM, self).__init__()
		self.name = 'transport_part_form_bin_to_agv_state'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(Cameras2SM, 'Cameras2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:37 y:662, x:1132 y:221
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.conveyor_belt_power = 100.0
		_state_machine.userdata.config_name_home = 'home'
		_state_machine.userdata.move_group = 'manipulator'
		_state_machine.userdata.move_group_prefix = '/ariac/arm1'
		_state_machine.userdata.action_topic = '/move_group'
		_state_machine.userdata.robot_name = ''
		_state_machine.userdata.camera_ref_frame = 'arm1_linear_arm_actuator'
		_state_machine.userdata.camera_topic = '/ariac/logical_camera_6'
		_state_machine.userdata.camera_frame = 'logical_camera_6_frame'
		_state_machine.userdata.part = 'gasket_part'
		_state_machine.userdata.config_name_bin6PreGrasp = 'bin6PreGrasp'
		_state_machine.userdata.tool_link = 'ee_link'
		_state_machine.userdata.part_offset = 0.04
		_state_machine.userdata.part_rotation = 0
		_state_machine.userdata.config_name_tray1PreDrop = 'tray1PreDrop'
		_state_machine.userdata.agv_pose = []
		_state_machine.userdata.part_offsetPre = 0.1
		_state_machine.userdata.part_offsetDown = 0.045
		_state_machine.userdata.arm = ''
		_state_machine.userdata.part_type = ''
		_state_machine.userdata.pose = ''
		_state_machine.userdata.arm1 = '/ariac/arm1'
		_state_machine.userdata.arm2 = '/ariac/arm2'
		_state_machine.userdata.bin4_pose = ''
		_state_machine.userdata.offset = 0
		_state_machine.userdata.rotation = 0
		_state_machine.userdata.config_name_bin4PreGrasp = 'bin4PreGrasp'
		_state_machine.userdata.config_name = home

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:298 y:114
			OperatableStateMachine.add('MoveR1Home',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Cameras2', 'planning_failed': 'WaitRetry1', 'control_failed': 'WaitRetry1', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:303 y:23
			OperatableStateMachine.add('WaitRetry1',
										WaitState(wait_time=1),
										transitions={'done': 'MoveR1Home'},
										autonomy={'done': Autonomy.Off})

			# x:865 y:119
			OperatableStateMachine.add('ComputePick',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'EnableGripper', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'arm1', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'part_offsetPre', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1283 y:121
			OperatableStateMachine.add('MoveR1ToPick1',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'ComputePick2', 'planning_failed': 'WaitRetry3', 'control_failed': 'WaitRetry3'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'arm1', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1683 y:117
			OperatableStateMachine.add('WaitRetry3',
										WaitState(wait_time=1),
										transitions={'done': 'MoveR1ToPick1'},
										autonomy={'done': Autonomy.Off})

			# x:1518 y:291
			OperatableStateMachine.add('WachtEven',
										WaitState(wait_time=2),
										transitions={'done': 'MoveR1PreGrasp2'},
										autonomy={'done': Autonomy.Off})

			# x:1489 y:392
			OperatableStateMachine.add('MoveR1PreGrasp2',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'MoveR1PreDrop', 'planning_failed': 'WaitRetry4', 'control_failed': 'WaitRetry4', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin6PreGrasp', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1665 y:391
			OperatableStateMachine.add('WaitRetry4',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreGrasp2'},
										autonomy={'done': Autonomy.Off})

			# x:1492 y:476
			OperatableStateMachine.add('MoveR1PreDrop',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'GetAgvPose', 'planning_failed': 'WaitRetry5', 'control_failed': 'WaitRetry5', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_tray1PreDrop', 'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1671 y:488
			OperatableStateMachine.add('WaitRetry5',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1PreDrop'},
										autonomy={'done': Autonomy.Off})

			# x:1507 y:559
			OperatableStateMachine.add('GetAgvPose',
										GetObjectPoseState(object_frame='kit_tray_1', ref_frame='arm1_linear_arm_actuator'),
										transitions={'continue': 'ComputeDrop', 'failed': 'ComputeDrop'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'agv_pose'})

			# x:1299 y:561
			OperatableStateMachine.add('ComputeDrop',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveR1ToDrop', 'failed': 'MoveR1ToDrop'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'arm1', 'tool_link': 'tool_link', 'pose': 'agv_pose', 'offset': 'part_offsetDown', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1282 y:484
			OperatableStateMachine.add('MoveR1ToDrop',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'DisableGripper', 'planning_failed': 'WaitRetry6', 'control_failed': 'WaitRetry6'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'arm1', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1157 y:561
			OperatableStateMachine.add('WaitRetry6',
										WaitState(wait_time=5),
										transitions={'done': 'MoveR1ToDrop'},
										autonomy={'done': Autonomy.Off})

			# x:1071 y:119
			OperatableStateMachine.add('EnableGripper',
										VacuumGripperControlState(enable=True, service_name='/ariac/arm1/gripper/control'),
										transitions={'continue': 'MoveR1ToPick1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1476 y:197
			OperatableStateMachine.add('ComputePick2',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveR1ToPick2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'arm1', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'part_offset', 'rotation': 'part_rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1273 y:269
			OperatableStateMachine.add('MoveR1ToPick2',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'WachtEven', 'planning_failed': 'WaitRetry7', 'control_failed': 'WaitRetry7'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'arm1', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:945 y:562
			OperatableStateMachine.add('DisableGripper',
										VacuumGripperControlState(enable=False, service_name='/ariac/arm1/gripper/control'),
										transitions={'continue': 'RetrunHomeR1', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:1676 y:239
			OperatableStateMachine.add('WaitRetry7',
										WaitState(wait_time=1),
										transitions={'done': 'MoveR1ToPick2'},
										autonomy={'done': Autonomy.Off})

			# x:944 y:634
			OperatableStateMachine.add('RetrunHomeR1',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'finished', 'planning_failed': 'Waitretry8', 'control_failed': 'Waitretry8', 'param_error': 'Waitretry8'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_home', 'move_group': 'move_group', 'move_group_prefix': 'arm1', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:1161 y:640
			OperatableStateMachine.add('Waitretry8',
										WaitState(wait_time=1),
										transitions={'done': 'RetrunHomeR1'},
										autonomy={'done': Autonomy.Off})

			# x:499 y:113
			OperatableStateMachine.add('Cameras2',
										self.use_behavior(Cameras2SM, 'Cameras2'),
										transitions={'finished': 'DetectArm', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part_type': 'part_type', 'arm': 'arm', 'part_pose': 'part_pose'})

			# x:682 y:172
			OperatableStateMachine.add('DetectArm',
										EqualState(),
										transitions={'true': 'ComputePick', 'false': 'ComputePickR2_1'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'value_a': 'arm', 'value_b': 'arm1'})

			# x:688 y:376
			OperatableStateMachine.add('EnableGripper2',
										VacuumGripperControlState(enable=True, service_name='/ariac/arm2gripper/control'),
										transitions={'continue': 'MoveR2ToPick2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:687 y:267
			OperatableStateMachine.add('ComputePickR2_1',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'EnableGripper2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'arm2', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:688 y:478
			OperatableStateMachine.add('MoveR2ToPick2',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'ComputePickR2_2', 'planning_failed': 'MoveR2ToPick2', 'control_failed': 'MoveR2ToPick2'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'move_group_prefix', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:689 y:609
			OperatableStateMachine.add('ComputePickR2_2',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'WachtEven2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:530 y:611
			OperatableStateMachine.add('WachtEven2',
										WaitState(wait_time=2),
										transitions={'done': 'MoveR2PreGrasp'},
										autonomy={'done': Autonomy.Off})

			# x:486 y:460
			OperatableStateMachine.add('MoveR2PreGrasp',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'GetBin4Pose', 'planning_failed': 'MoveR2PreGrasp', 'control_failed': 'MoveR2PreGrasp', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name_bin4PreGrasp', 'move_group': 'move_group', 'move_group_prefix': 'arm2', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:299 y:488
			OperatableStateMachine.add('GetBin4Pose',
										GetObjectPoseState(object_frame='kit_tray_1', ref_frame='arm2_linear_arm_actuator'),
										transitions={'continue': 'ComputeDropR2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'bin4_pose'})

			# x:116 y:487
			OperatableStateMachine.add('ComputeDropR2',
										ComputeGraspAriacState(joint_names=['linear_arm_actuator_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']),
										transitions={'continue': 'MoveR2ToDrop', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'tool_link': 'tool_link', 'pose': 'pose', 'offset': 'offset', 'rotation': 'rotation', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:115 y:364
			OperatableStateMachine.add('MoveR2ToDrop',
										MoveitToJointsDynAriacState(),
										transitions={'reached': 'DisableGripper2', 'planning_failed': 'MoveR2ToDrop', 'control_failed': 'MoveR2ToDrop'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'move_group_prefix': 'move_group_prefix', 'move_group': 'move_group', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:116 y:256
			OperatableStateMachine.add('DisableGripper2',
										VacuumGripperControlState(enable=True, service_name='/ariac/arm2/gripper/control'),
										transitions={'continue': 'ReturnHomeR2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:302 y:256
			OperatableStateMachine.add('ReturnHomeR2',
										SrdfStateToMoveitAriac(),
										transitions={'reached': 'Cameras2', 'planning_failed': 'ReturnHomeR2', 'control_failed': 'ReturnHomeR2', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'move_group_prefix': 'move_group_prefix', 'action_topic': 'action_topic', 'robot_name': 'robot_name', 'config_name_out': 'config_name_out', 'move_group_out': 'move_group_out', 'robot_name_out': 'robot_name_out', 'action_topic_out': 'action_topic_out', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
