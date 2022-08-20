#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import argparse

class talker_play:
    def __init__(self, topic_name = 'chatter'):
        rospy.init_node('talker', anonymous=True)
        self.topic_name = topic_name
        self.pub = rospy.Publisher(topic_name, String, queue_size=10)

    def pub_string(self):
        hello_str = str("hello world (%s) %s" % (self.topic_name, rospy.get_time()))
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)

    def talker(self):        
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.pub_string()
            rate.sleep()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-t', '--topic_name', default='chatter',
        help='define topic_name'
    )

    args = parser.parse_args(rospy.myargv()[1:])

    topic_name = args.topic_name
    obj = talker_play(topic_name)
    try:
        obj.talker()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    
    main()
