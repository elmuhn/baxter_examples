#!/usr/bin/python2

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import argparse

import rospy
import xacro_jade

from baxter_core_msgs.msg import (
    URDFConfiguration,
)

def xacro_parse(filename):
    doc = xacro_jade.parse(None, filename)
    xacro_jade.process_doc(doc, in_order=True)
    return doc.toprettyxml(indent='  ')

def send_urdf(parent_link, root_joint, urdf_filename):
    """
    Send the URDF Fragment located at the specified path to
    modify the electric gripper on Baxter.

    @param parent_link: parent link to attach the URDF fragment to
                        (usually <side>_hand)
    @param root_joint: root link of the URDF fragment (usually <side>_gripper_base)
    @param urdf_filename: path to the urdf XML file to load into xacro and send
    """
    msg = URDFConfiguration()
    msg.time = rospy.Time.now()
    msg.link = parent_link
    msg.joint = root_joint
    msg.urdf = xacro_parse(urdf_filename)
    pub = rospy.Publisher('/robot/urdf', URDFConfiguration, queue_size=10)
    rate = rospy.Rate(5) # 5hz
    # Sleep to allow for URDF Fragment to be published.
    rospy.sleep(1)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

def main():
    """RSDK End Effector URDF Fragment Example: 
    
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='Path to URDF file to send'
    )
    required.add_argument(
        '-s', '--side', required=True,
        help='End Effector Side <left/right>'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('rsdk_configure_end_effector', anonymous=True)

    if not os.access(args.file, os.R_OK):
        rospy.logerr("Cannot read file at '%s'" % (args.file,))
        return 1
    send_urdf("{}_hand".format(args.side),
              "{}_gripper_base".format(args.side),
              args.file)
    return 0

if __name__ == '__main__':
    sys.exit(main())
