#!/usr/bin/env python

import rospy
from json_prolog import json_prolog
import suturo_perception_msgs.msg as message
from std_msgs.msg import ColorRGBA

prolog = json_prolog.Prolog()


def callback(perceived_object_list):
    for data in perceived_object_list.result.detectionData:
        rospy.wait_for_service('/json_prolog/query')

        obj_class = str(data.obj_class)
        if obj_class and float(data.confidence) > 0.5:
            obj_class = obj_class.capitalize().replace('_', '')
        else:
            rospy.loginfo("The given class name is empty. Setting to OTHER.")
            obj_class = "Other"

        # confidence = '1.0' if data.confidence == 0.0 else data.confidence
        # shape = str(data.shape)
        source_frame = 'map'
        depth = str(data.depth)
        width = str(data.width)
        height = str(data.height)
        r = str(data.color.r)
        g = str(data.color.g)
        b = str(data.color.b)
        a = str(data.color.a)

        x = str(data.pose.pose.position.x)
        y = str(data.pose.pose.position.y)
        z = str(data.pose.pose.position.z)
        qx = str(data.pose.pose.orientation.x)
        qy = str(data.pose.pose.orientation.y)
        qz = str(data.pose.pose.orientation.z)
        qw = str(data.pose.pose.orientation.w)
        threshold = "0.05"
        region_splits = str(data.region).split('_')
        surface_query = ""
        if str(data.region).endswith("table"):
            surface_query = 'table_surface(Surface),'
        elif "shelf" in region_splits:
            surface_query = "rdf_equal(Surface, 'http://knowrob.org/kb/robocup.owl#kitchen_description_shelf_floor_%s_piece')," \
                            % region_splits.pop()
        else:
            rospy.loginfo("Issue with region: %s" % data.region)

        filter_plane_noise_query = "{}" \
                                   "surface_pose_in_map(Surface, [[_,_,Z],_]), " \
                                   "ZOffset is {} - Z," \
                                   "ZOffset > 0.025,".format(surface_query, z)

        plane_solutions_raw = prolog.all_solutions(filter_plane_noise_query)
        print(plane_solutions_raw)
        if plane_solutions_raw:
            print("Object is at valid height")
            query_string = (surface_query +
                            "create_object_at(hsr_objects:'" +
                            obj_class + "'," +
                            "['" + source_frame +
                            "', _, [" + ", ".join([x,y,z]) + "]," +
                            "[" + ", ".join([qx,qy,qz,qw]) + "]]," +
                            threshold + ", ObjectInstance," +
                            "[" + ", ".join([depth,width,height]) + "], " +
                            "[" + ", ".join([r,g,b,a]) + "])," +
                            "assert_object_on(ObjectInstance, Surface).")
            rospy.loginfo('Send query: \n' + query_string)
            solutions = prolog.all_solutions(query_string)
            rospy.loginfo(solutions)
        else:
            print("Invalid Z-pose of the object. IGNORING the object")


def listener():
    rospy.init_node('perception_listener', anonymous=False)
    rospy.Subscriber("extract_object_infos/result", message.ExtractObjectInfoActionResult, callback)
    # PerceptionSubscriber('table', "hsr_perception_table/result", message.PerceiveTableActionResult)
    # PerceptionSubscriber('shelf', "hsr_perception_shelf/result", message.PerceiveShelfActionResult)
    rospy.spin()


if __name__ == '__main__':
    listener()


# ------- Old subscriber for 2 pipelines
#
# class PerceptionSubscriber(rospy.Subscriber):
#     def __init__(self, sem_location, topic, message_type):
#         self.sem_location = sem_location
#         super(PerceptionSubscriber, self).__init__(topic, message_type, self.callback)
#         self.surface_query = ''
#
#     def callback(self, perceived_object_list):
#         for data in perceived_object_list.result.detectionData:
#             rospy.wait_for_service('/json_prolog/query')
#
#             obj_class = str(data.obj_class)
#             if obj_class:
#                 obj_class = obj_class.capitalize().replace('_', '')
#             else:
#                 rospy.loginfo("The given class name is empty. Setting to OTHER.")
#                 obj_class = "Other"
#
#             confidence = '1.0' if data.confidence == 0.0 else data.confidence
#             shape = str(data.shape)
#             source_frame = 'map'
#
#             x = str(data.pose.pose.position.x)
#             y = str(data.pose.pose.position.y)
#             z = str(data.pose.pose.position.z)
#             qx = str(data.pose.pose.orientation.x)
#             qy = str(data.pose.pose.orientation.y)
#             qz = str(data.pose.pose.orientation.z)
#             qw = str(data.pose.pose.orientation.w)
#             threshold = "0.05"
#
#             if self.sem_location == 'table':
#                 self.surface_query = 'table_surface(Surface),'
#             elif self.sem_location == 'shelf':
#                 self.surface_query = "shelf_floor_at_height(" + z + ", Surface),"
#
#             query_string = ("create_object_at(hsr_objects:'" +
#                             obj_class + "'," +
#                             "['" + source_frame +
#                             "', _, [" + x + "," + y + "," + z + "]," +
#                             "[" + qx + "," + qy + "," + qz + "," + qw + "]]," +
#                             threshold + ", ObjectInstance)," +
#                             self.surface_query +
#                             "assert_object_on(ObjectInstance, Surface).")
#             rospy.loginfo('Send query: \n' + query_string)
#             solutions = prolog.all_solutions(query_string)
#             rospy.loginfo(solutions)

