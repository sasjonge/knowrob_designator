#!/usr/bin/env python

import rospy
from knowrob_designator.srv import SimpleQuery, SimpleQueryResponse
import json

class DesignatorService:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('DesignatorService')
        
        # Define the ROS service with the custom service type
        self.service = rospy.Service('simple_query', SimpleQuery, self.handle_query)
        rospy.loginfo("KnowRobDesignatorService is ready to receive queries.")

    def handle_query(self, request):
        # Extract the query from the request
        query = request.query
        rospy.loginfo(f"Received query: {query}")

        if "type=\"Milk\"" in query and "storagePlace=\"?storagePlace\"" in query:
            # Respond to the storage place query
            response = {
                "?storagePlace": "Fridge",
                "?spatialRelation": "in"
            }

        elif "type=\"Fridge\"" in query and "handle=\"?handle\"" in query:
            # Respond to the handle query
            response = {
                "?handle": "frdige_door_link_handle",  # Replace with actual base link frame from semantic map
                "?open": True
            }
        else:
            # Default response if the query doesn't match
            response = {"error": "Unknown query format or unsupported query type."}

        # Convert the response to JSON format
        response_json = json.dumps(response)
        
        # Return the JSON string in the custom service response
        return SimpleQueryResponse(response=response_json)

    def run(self):
        # Keep the service running
        rospy.spin()

if __name__ == '__main__':
    try:
        service = DesignatorService()
        service.run()
    except rospy.ROSInterruptException:
        pass
