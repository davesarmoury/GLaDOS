#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from openai_ros.srv import Completion, CompletionResponse
import re
import num2words

def replace_numbers(text):
    return re.sub(r"(\d+)", lambda x: num2words.num2words(int(x.group(0))), text)

def callback(msg):
    global chat_service, pub
    resp = chat_service(msg.data, 1.0)

    text = resp.text.replace('\n', ' ')     # Remove line-breaks
    text = re.sub(r"\((.*?)\)", " ", text)  # Remove anything in brackets
    text = replace_numbers(text)            # Make digits into text
    text = re.sub(' +', ' ', text)          # Unnecessary white space

    pub.publish(text)

def main():
    global chat_service, pub
    rospy.init_node('glados_coordinator')

    rospy.wait_for_service('/get_response')
    chat_service = rospy.ServiceProxy('/get_response', Completion)
    pub = rospy.Publisher('/speak', String, queue_size=10)

    rospy.Subscriber("/final", String, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass