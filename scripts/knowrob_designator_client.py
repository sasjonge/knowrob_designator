#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Test client for DesignatorResolveStart action

import rospy
import actionlib
from knowrob_designator.msg import DesignatorResolutionStartAction, DesignatorResolutionStartGoal
from time import time

def main():
    rospy.init_node('designator_resolve_test_client')

    client = actionlib.SimpleActionClient('/knowrob/designator_resolving_started', DesignatorResolutionStartAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()

    goal = DesignatorResolutionStartGoal()
    goal.designator_id = "desig_001"
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
    goal.stamp = rospy.Time.now()

    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"Result: success={result.success}, message='{result.message}'")

if __name__ == '__main__':
    main()
