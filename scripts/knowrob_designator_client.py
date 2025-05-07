#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import actionlib
from knowrob_designator.msg import LogDesignatorAction, LogDesignatorGoal

rospy.init_node('designator_test_client')
client = actionlib.SimpleActionClient('/knowrob_designator/log', LogDesignatorAction)
client.wait_for_server()

goal = LogDesignatorGoal()
goal.json_designator = """
{
  "anAction": {
    "type": "Transporting",
    "objectActedOn": {
      "anObject": {
        "type": "Milk"
      }
    },
    "target": {
      "theLocation": {
        "goal": {
          "theObject": {
            "name": "Table1"
          }
        }
      }
    }
  }
}
"""

client.send_goal(goal)
client.wait_for_result()
print(client.get_result())
