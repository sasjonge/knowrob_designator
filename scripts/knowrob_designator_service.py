#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
knowrob_designator_service.py

This script starts a ROS action server that accepts a JSON-encoded "designator"
(i.e., a structured symbolic description of a situation, action, or object)
and logs its semantic content as RDF-style triples into the KnowRob knowledge base
using the 'tell' interface.

The conversion from nested designator to triples is performed by the DesignatorParser class.

Author: Sascha Jongebloed
"""

import rospy
import json
import actionlib

from knowrob_designator.msg import LogDesignatorAction, LogDesignatorResult, LogDesignatorFeedback
from knowrob_ros.knowrob_ros_lib import KnowRobRosLib, TripleQueryBuilder, get_default_modalframe
from designator_parser import DesignatorParser


class LogDesignatorServer:
    """
    Action server that receives a JSON-formatted designator, parses it into RDF triples,
    and asserts it into the KnowRob knowledge base using the tell interface.

    The action server listens on: /knowrob_designator/log
    """

    def __init__(self):
        """
        Initializes the action server and supporting tools:
        - KnowRobRosLib: for communicating with KnowRob via actionlib.
        - DesignatorParser: to convert designator structures to triples.
        """
        self.server = actionlib.SimpleActionServer(
            '/knowrob_designator/log',          # Action name
            LogDesignatorAction,                # Action definition
            self.execute_log,                   # Callback for execution
            False                                # Do not start immediately
        )

        self.knowrob = KnowRobRosLib()
        self.knowrob.init_clients()             # Set up action clients for KnowRob

        self.parser = DesignatorParser()        # Stateless parser for designators
        self.server.start()                     # Start the action server

        rospy.loginfo("LogDesignator action server started.")

    def execute_log(self, goal):
        """
        Callback function for the action server.

        It takes the designator (as a JSON string), parses it into triples,
        and sends those triples to KnowRob. The result is reported via action result.

        Args:
            goal (LogDesignatorGoal): contains a JSON string with the designator structure
        """
        feedback = LogDesignatorFeedback()
        result = LogDesignatorResult()

        try:
            # Parse incoming JSON string to dictionary
            designator = json.loads(goal.json_designator)

            # Convert to RDF-style triples
            triples = self.parser.parse(designator)

            # Package into ROS Triple messages
            builder = TripleQueryBuilder()
            for s, p, o in triples:
                builder.add(s, p, o)

            feedback.status = f"Sending {len(triples)} triples to KnowRob..."
            self.server.publish_feedback(feedback)

            # Send triples to KnowRob via tell action
            tell_result = self.knowrob.tell(builder.get_triples(), get_default_modalframe())
            result.success = (tell_result.status == 1)
            result.message = "Success" if result.success else "Failed to log to KnowRob"

        except Exception as e:
            # Handle parsing or communication failure
            result.success = False
            result.message = f"Exception: {str(e)}"

        # Signal completion to the action client
        self.server.set_succeeded(result)


if __name__ == '__main__':
    print("Starting KnowRob Designator Service...")
    rospy.init_node('knowrob_designator')  # Initialize the ROS node

    print("Initializing KnowRob Designator Server...")
    server = LogDesignatorServer()

    print("KnowRob Designator Server is running...")
    rospy.spin()  # Keep the node alive
