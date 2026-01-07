#!/usr/bin/env python3
'''
 * File Name: rfh_tags_localization_pose_shifter.py
 *
 * Description: Shift a pose by the transition and rotation between sourceFrameId and targetFrameId in a global frame. 
 *              For example, when we collect a pose of camera(frame is camera) of a handheld, the pose is in a global frame.
 *              we need the pose of the antenna (the frame is antenna) of the handheld.
 *              This file provides a method to shift the pose of camera to get the antenna's pose.
 *
 * Author: Jian Zhang
 *
 * Create Date: 2017-10-2
 *
 * Modified Date: Nov/10/2025 by Justin Palm
 *
 * Version: 2.0 for ROS2 Humble
 * Remark:
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import tf_transformations
import math
import time


# tf transform still not working !
class rfhposeShifter:
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        # tf buffer length (cache)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.br = tf2_ros.TransformBroadcaster(self.node)

    def shiftPose(self, sourceFrameId, targetFrameId, globalFrameId, pose):
        '''
        * Description: It can work when the camera localization is on or off.
        *              Shift a pose by the transition and rotation between sourceFrameId and targetFrameId.
        *              For example, we shift a pose of camera(frame is sourceFrameId) of a handheld, 
        *              to get the pose of the antenna (the frame is targetFrameId) of the handheld.
        *              The two poses are in coordinate of globalFrame.
        *              
        *               The current algorithm is: create a transform(tfSource2Global) on recorded pose, 
        *               Then create a transform and set the tfSource2Global and transform of sourceFrameId
        *               to targetFrameId. Last we lookup transform (trans, rot) of globalFrameId to targetFrameId.
        *               This transform (trans, rot) is the antenna pose in global frame.
        *               
        *               The old algorithm is: we listen the tf(s2a_tf) of sourceFrameId(zedcenter) to targetFrameId(antenna)  
        *               this s2a_tf is the transform from sourceFrameId to targetFrameId in coordinate of sourceFrameId
        *               then we convert the t2s_tf in globalFrameId (g2a_tf), g2a_tf is the transform of pose in globalframe
        *               to antenna, so the pose of the antenna can be get by shifting input pose with g2a_tf.translation
        *               and rotating with g2a_tf.rotation
        * Input:  sourceFrameId: the source frame id 
                  targetFrameId: the target frame id
                  globalFrameId: the global frame id
                  pose: the pose of sourceFrameId in coordinate of globalFrameId 
        * Output: newPose, the pose of targetFrameId in coordinate of globalFrameId 
        '''
        newPose = None
        try:       
            # get the tf of transforming sourceFrameId to targetFrameId in the coordinate of sourceFrameId         
            transform = self.tf_buffer.lookup_transform(
                sourceFrameId,
                targetFrameId, 
                rclpy.time.Time(),  # get the tf at first available time because this tf is fixed
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # The tf should be based on the recorded pose  
            # provide the tf of transforming header.frame_id to child_frame_id in coordinate of header.frame_id
            tfSource2Global = TransformStamped()  # creating new transform msg
            tfSource2Global.header.stamp = self.get_clock().now().to_msg()
            tfSource2Global.header.frame_id = globalFrameId 
            tfSource2Global.child_frame_id = sourceFrameId 
            tfSource2Global.transform.translation = pose.pose.pose.position
            tfSource2Global.transform.rotation = pose.pose.pose.orientation

            # emulate composition (since TransformerROS is not in ROS2)
            trans = [
                tfSource2Global.transform.translation.x + transform.transform.translation.x,
                tfSource2Global.transform.translation.y + transform.transform.translation.y,
                tfSource2Global.transform.translation.z + transform.transform.translation.z,
            ]
            rot = tf_transformations.quaternion_multiply(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ],
                [
                    tfSource2Global.transform.rotation.x,
                    tfSource2Global.transform.rotation.y,
                    tfSource2Global.transform.rotation.z,
                    tfSource2Global.transform.rotation.w,
                ],
            )

            newPose = PoseStamped()
            newPose.header.frame_id = globalFrameId
            newPose.pose.position.x = trans[0]
            newPose.pose.position.y = trans[1]
            newPose.pose.position.z = trans[2]
            newPose.pose.orientation.x = rot[0]
            newPose.pose.orientation.y = rot[1]
            newPose.pose.orientation.z = rot[2]
            newPose.pose.orientation.w = rot[3]

            # -------------------------------
            # Legacy commented approach (kept for reference)
            # -------------------------------
            # if the current pose and tf are consistent we can lookup transform
            # tfSource2Global = self.tf_buffer.lookup_transform(
            #     sourceFrameId, globalFrameId, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))

            # # based on the transform to create a point(tfpoint) with transform (x,y,z) in source frame,
            # tfS2TinSourceFramePoint = PointStamped() 
            # tfS2TinSourceFramePoint.point.x = transform.transform.translation.x 
            # tfS2TinSourceFramePoint.point.y = transform.transform.translation.y 
            # tfS2TinSourceFramePoint.point.z = transform.transform.translation.z
            # tfS2TinGlobalFramePoint = tf2_geometry_msgs.do_transform_point(tfS2TinSourceFramePoint, tfSource2Global)

            # # get the position of original point in global frame
            # sOriginSource = PointStamped()
            # sOriginSource.point.x = 0
            # sOriginSource.point.y = 0
            # sOriginSource.point.z = 0
            # sOrigininGlobal = tf2_geometry_msgs.do_transform_point(sOriginSource, tfSource2Global)

            # # delta position between tfpoint and original point in global frame
            # tfDeltaX = tfS2TinGlobalFramePoint.point.x - sOrigininGlobal.point.x
            # tfDeltaY = tfS2TinGlobalFramePoint.point.y - sOrigininGlobal.point.y
            # tfDeltaZ = tfS2TinGlobalFramePoint.point.z - sOrigininGlobal.point.z
            # # deltaDis = (tfDeltaX**2 + tfDeltaY**2 + tfDeltaZ**2)**0.5
            
            # newPose = PoseStamped()
            # newPose.header.frame_id = globalFrameId
            # newPose.pose.position.x = pose.pose.pose.position.x + tfDeltaX
            # newPose.pose.position.y = pose.pose.pose.position.y + tfDeltaY
            # newPose.pose.position.z = pose.pose.pose.position.z + tfDeltaZ

            # euler_p = tf_transformations.euler_from_quaternion([
            #     pose.pose.pose.orientation.x,
            #     pose.pose.pose.orientation.y,
            #     pose.pose.pose.orientation.z,
            #     pose.pose.pose.orientation.w])
            # euler_t = tf_transformations.euler_from_quaternion([
            #     transform.transform.rotation.x,
            #     transform.transform.rotation.y,
            #     transform.transform.rotation.z,
            #     transform.transform.rotation.w])
            # rollnew = euler_p[0] + euler_t[0]
            # pitchnew = euler_p[1] + euler_t[1]
            # yawnew = euler_p[2] + euler_t[2]
            # qat = tf_transformations.quaternion_from_euler(rollnew, pitchnew, yawnew)
            # newPose.pose.orientation.x = qat[0]
            # newPose.pose.orientation.y = qat[1]
            # newPose.pose.orientation.z = qat[2]
            # newPose.pose.orientation.w = qat[3]

            # -------------------------------
            # Debug visualization (works in ROS2)
            # -------------------------------
            # self.br.sendTransform(TransformStamped(
            #     header=self._make_header('map'),
            #     child_frame_id="zed_center_1",
            #     transform=tfSource2Global.transform
            # ))
            # self.br.sendTransform(TransformStamped(
            #     header=self._make_header('map'),
            #     child_frame_id="antenna_1",
            #     transform=TransformStamped(
            #         transform.translation=pose.pose.pose.position,
            #         transform.rotation=pose.pose.pose.orientation
            #     ).transform
            # ))

        except Exception as e:
            self.get_logger().warn(f'unable to do transform [{e}]')
        
        return newPose

    def shiftPoseStatic(self, sourceFrameId, targetFrameId, globalFrameId, pose):
        '''
        * Description: Without the support handheld/camera tf, and can not work when the camera localization is on.
        *              Shift a pose by the transition and rotation between sourceFrameId and targetFrameId.
        *              For example, we shift a pose of camera(frame is sourceFrameId) of a handheld, 
        *              to get the pose of the antenna (the frame is targetFrameId) of the handheld.
        *              The two poses are in coordinate of globalFrame.
        *              
        *              The algorithm is create a new transform based on recorded pose, and broadcast it,
        *              then listen the tf between targetFrameId and globalFrameId, this tf is the pose of antenna (targetFrameId)
        * Input:  sourceFrameId: the source frame id 
                  targetFrameId: the target frame id
                  globalFrameId: the global frame id
                  pose: the pose of sourceFrameId in coordinate of globalFrameId 
        * Output: newPose, the pose of targetFrameId in coordinate of globalFrameId 
        '''
        newPose = None
        try:     
            # for debug broadcast
            t = TransformStamped()
            euler_p = tf_transformations.euler_from_quaternion([
                pose.pose.pose.orientation.x,
                pose.pose.pose.orientation.y,
                pose.pose.pose.orientation.z,
                pose.pose.pose.orientation.w])
            quat = tf_transformations.quaternion_from_euler(euler_p[0], euler_p[1], euler_p[2])
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = globalFrameId
            t.child_frame_id = sourceFrameId
            t.transform.translation = pose.pose.pose.position
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.br.sendTransform(t)

            time.sleep(0.5)

            # lookup_transform provide the tf from target_frame(globalFrameId) to source_frame(targetFrameId)
            tfAntenna2Global = self.tf_buffer.lookup_transform(
                globalFrameId,
                targetFrameId, 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            newPose = PoseStamped()
            newPose.header.frame_id = globalFrameId
            newPose.pose.position = tfAntenna2Global.transform.translation
            newPose.pose.orientation = tfAntenna2Global.transform.rotation
                     
        except Exception as e:
            self.get_logger().warn(f'unable to do transform [{e}]')

        return newPose


def main(args=None):
    #rclpy.init(args=args)
    node = rfhposeShifter()
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

            
