import hebi
import numpy as np
from time import sleep

def get_group():
    families = ['arm']
    names = ["Base", 'Elbow']
    lookup = hebi.Lookup()
    sleep(2.0)
    group = lookup.get_group_from_names(families, names)
    if group is None:
        print('inititalize_hebi: no group found')
        return None
    
    # Set gains
    gains_command = hebi.GroupCommand(group.size)
    try:
        gains_command.read_gains("gains/default_gains.xml")
    except:
        print('inititalize_hebi: failed to read gains')
        return None
    if not group.send_command_with_acknowledgement(gains_command):
        print('inititalize_hebi: failed to receive ack from group')
        return None
    return group

def initialize_hebi():
    group = get_group()    
    feedback = hebi.GroupFeedback(group.size)
    num_joints = group.size
    command = hebi.GroupCommand(num_joints)
    return group, feedback, command

def get_hebi_feedback(group, hebi_feedback, joint_offsets=np.array([-np.pi/2, 0.3]), limit_stop=True, limits=np.array([np.pi+0.09, 2*np.pi-0.09])):
    # joint_offsets - set in radians to change x/y position [rotation from hebi x, rotation from hebi y]
    # limit_stop currently limits position of both joints the same [lower limit, higher limit] in radians
    limit_stop_flag = False
    group.get_next_feedback(reuse_fbk=hebi_feedback)
    theta = np.array(hebi_feedback.position)
    omega = np.array(hebi_feedback.velocity)
    torque = np.array(hebi_feedback.effort)
    theta -= joint_offsets
    if (theta > limits).any():
        limit_stop_flag = True
    return theta, omega, torque, limit_stop_flag

def send_hebi_effort_command(group, command):
    group.send_command(command)
    return

def send_hebi_position_command(group, command, joint_offsets=np.array([-np.pi/2, 0.3])):
    # joint offset - set in radians to change x/y position
    # length of joint offsets must be consistent with number of joints
    offset_command = command.position
    offset_command += joint_offsets
    command.position = offset_command.tolist()
    group.send_command(command)
    return
    
