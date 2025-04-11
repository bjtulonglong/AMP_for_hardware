# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin
import glob

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

MOTION_FILES = glob.glob('datasets/mocap_motions/*')


class T1AMPCfg( LeggedRobotCfg ):

    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        include_history_steps = None  # Number of steps of history to include.
        num_observations = 48
        num_privileged_obs = 51
        num_actions = 13
        env_spacing = 1.0
        reference_state_initialization = True
        reference_state_initialization_prob = 0.85
        amp_motion_files = MOTION_FILES

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.72] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'Waist': 0.,
            'Left_Hip_Pitch': -0.2,
            'Left_Hip_Roll': 0.,
            'Left_Hip_Yaw': 0.,
            'Left_Knee_Pitch': 0.4,
            'Left_Ankle_Pitch': -0.25,
            'Left_Ankle_Roll': 0.,
            'Right_Hip_Pitch': -0.2,
            'Right_Hip_Roll': 0.,
            'Right_Hip_Yaw': 0.,
            'Right_Knee_Pitch': 0.4,
            'Right_Ankle_Pitch': -0.25,
            'Right_Ankle_Roll': 0.,
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'Waist': 200.0, 'Hip': 200.0, 'Knee': 200.0,
                     'Ankle': 100.0}
        damping = {'Waist': 2.0, 'Hip': 2.0, 'Knee': 2.0,
                     'Ankle': 1.0}
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'plane'
        measure_heights = False

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/t1/T1_locomotion.urdf'
        foot_name = "foot_link"
        knee_name = 'Shank'
        penalize_contacts_on = ["Trunk"]
        terminate_after_contacts_on = ["Trunk"]
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
        flip_visual_attachments = False
  
    class domain_rand:
        randomize_friction = True
        friction_range = [0.25, 1.75]
        randomize_base_mass = True
        added_mass_range = [-1., 1.]
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.0
        randomize_gains = True
        stiffness_multiplier_range = [0.9, 1.1]
        damping_multiplier_range = [0.9, 1.1]

    class noise:
        add_noise = True
        noise_level = 1.0 # scales other values
        class noise_scales:
            dof_pos = 0.03
            dof_vel = 1.5
            lin_vel = 0.1
            ang_vel = 0.3
            gravity = 0.05
            height_measurements = 0.1

    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.68
        class scales( LeggedRobotCfg.rewards.scales ):
            termination = 0.0
            tracking_lin_vel = 75
            tracking_lin_y_vel = -1
            tracking_ang_vel = -1
            # tracking_lin_vel = 1.5
            # tracking_ang_vel = 0.5
            survival = 0.0 # 存活
            lin_vel_z = -0.0
            ang_vel_xy = -0.0
            orientation = -0.0
            torques = -0.0000025
            dof_vel = -0.0
            dof_acc = -0.0
            base_height = -1.0 
            feet_air_time = 0.0
            
            feet_stumble = 0.0 
            action_rate = -0.0
            stand_still = 0.0
            dof_pos_limits = -1.0
            # add
            # torque_tiredness = -1.e-2
            # dof_pos_ref= -1.e-2
            # root_acc= -1.e-4
            # collision = 0.0
            # waist_pos= -1.
            # feet_slip = -0.1
            # feet_vel_z = -0.1
            # feet_yaw_diff = -1.
            # feet_yaw_mean = -1.
            # feet_roll = -0.1
            feet_distance = -5.
            # feet_swing = 2.5
            # stand_still = -1.


    class commands:
        curriculum = False
        max_curriculum = 1.
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = False # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [0.0, 1.0] # min max [m/s]
            lin_vel_y = [-0.0, 0.0]   # min max [m/s]
            ang_vel_yaw = [-0.0, 0.0]    # min max [rad/s]
            heading = [-3.14, 3.14]

class T1AMPCfgPPO( LeggedRobotCfgPPO ):
    runner_class_name = 'AMPOnPolicyRunner'
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.0
        amp_replay_buffer_size = 1000000
        num_learning_epochs = 5
        num_mini_batches = 4
        disc_coef = 5  # 5
        bounds_loss_coef = 2

    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 't1_amp_example'
        algorithm_class_name = 'AMPPPO'
        policy_class_name = 'ActorCritic'
        max_iterations = 300000 # number of policy updates

        amp_reward_coef = 2.0
        amp_motion_files = MOTION_FILES
        amp_num_preload_transitions = 2000000
        amp_task_reward_lerp = 0.5
        amp_discr_hidden_dims = [1024, 512]

        disc_grad_penalty = 5 # original 10

        min_normalized_std = [0.05, 0.05, 0.05,0.05, 0.05, 0.05,0.05, 0.05, 0.05,0.05, 0.05, 0.05,0.05]

  