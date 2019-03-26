#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
import suturo_perception_msgs.msg as message

prolog = json_prolog.Prolog()


class PerceptionSubscriber(rospy.Subscriber):
    def __init__(self, sem_location, topic, message_type):
        self.sem_location = sem_location
        super(PerceptionSubscriber, self).__init__(topic, message_type, self.callback)

    def callback(self, perceived_object_list):
        for data in perceived_object_list.result.detectionData:
            rospy.wait_for_service('/json_prolog/query')

            obj_class = str(data.obj_class)
            if obj_class:
                obj_class = obj_class.capitalize().replace('_', '')
            else:
                rospy.loginfo("The given class name is empty. Setting to OTHER.")
                obj_class = "Other"

            confidence = '1.0' if data.confidence == 0.0 else data.confidence
            shape = str(data.shape)
            source_frame = 'map'

            x = str(data.pose.pose.position.x)
            y = str(data.pose.pose.position.y)
            z = str(data.pose.pose.position.z)
            qx = str(data.pose.pose.orientation.x)
            qy = str(data.pose.pose.orientation.y)
            qz = str(data.pose.pose.orientation.z)
            qw = str(data.pose.pose.orientation.w)
            threshold = "0.05"

            self.surface_query = ''
            if self.sem_location == 'table':
                self.surface_query = 'table_surface(Surface),'
            elif self.sem_location == 'shelf':
                self.surface_query = "rdf_equal(Surface, robocup:'kitchen_description_shelf_floor_2_piece')," ### <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

            query_string = ("create_object_at(hsr_objects:'" +
                            obj_class + "'," +
                            "['" + source_frame +
                            "', _, [" + x + "," + y + "," + z + "]," +
                            "[" + qx + "," + qy + "," + qz + "," + qw + "]]," +
                            threshold + ", ObjectInstance)," +
                            self.surface_query +
                            "assert_object_on(ObjectInstance, Surface).")
            rospy.loginfo('Send query: \n' + query_string)
            solutions = prolog.all_solutions(query_string)
            rospy.loginfo(solutions)


def listener():
    rospy.init_node('perception_listener', anonymous=True)
    PerceptionSubscriber('table', "hsr_perception_table/result", message.PerceiveTableActionResult)
    PerceptionSubscriber('shelf', "hsr_perception_shelf/result", message.PerceiveShelfActionResult)
    rospy.spin()


if __name__ == '__main__':
    listener()
