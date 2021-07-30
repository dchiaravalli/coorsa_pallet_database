#!/usr/bin/env python
# coding=utf-8
from pallet_handler import PalletHandler
from pallet_database_pkg.srv import *
from geometry_msgs.msg import Pose
import rospy
import copy

class PalletDatabase:

    def __init__(self, pallet_file_path):
        self.pallet_handler = PalletHandler(pallet_file_path)
        rospy.init_node('pallet_database')

        pallet_info_service = rospy.Service('pallet_database/get_pallet_info', pallet_info, self.pallet_info_cb)
        box_detected_service = rospy.Service('pallet_database/save_box_detected', box_detected, self.box_detected_cb)
        box_request_service = rospy.Service('pallet_database/get_box_for_recovery', box_recovery, self.box_recovery_cb)
        box_confirm_service = rospy.Service('pallet_database/confirm_box_recovery', box_confirm, self.box_confirm_cb)

        rospy.spin()

    def pallet_info_cb(self,req):
        res = pallet_infoResponse()
        data = self.pallet_handler.getPalletInfo(req.box_type)
        res.pallet_name = data['name']
        res.pallet_pose.position.x = data['pose'][0]
        res.pallet_pose.position.y = data['pose'][1]
        res.pallet_pose.position.z = data['pose'][2]
        res.pallet_dim.position.x = data['dim'][0]
        res.pallet_dim.position.y = data['dim'][1]
        res.pallet_dim.position.z = data['dim'][2]
        res.box_dim.position.x = data['box_dim'][0]
        res.box_dim.position.y = data['box_dim'][1]
        res.box_dim.position.z = data['box_dim'][2]
        res.box_detected = data['box_detected']
        return res

    def box_detected_cb(self,req):
        res = box_detectedResponse()
        res.acknowledged = True
        data = {}
        data['name'] = req.pallet_name
        data['plane'] = [req.plane_detected.data[0],req.plane_detected.data[1],req.plane_detected.data[2],req.plane_detected.data[3]]
        for item in req.box_detected:
            data['box_list'].append([item.position.x, item.position.y, item.position.z]) 
        self.pallet_handler.setPalletDetectionData(data)
        return res

    def box_recovery_cb(self,req):
        res = box_recoveryResponse()
        box_list = self.pallet_handler.getBoxPoses(req.box_number)
        temp = Pose()
        if not box_list:
            res.box_available = False
            return res
        for item in box_list:
            temp.position.x = item[0]
            temp.position.y = item[1]
            temp.position.z = item[2]
            res.box_list.append(copy.deepcopy(temp))
            res.box_available = True
        return res
    
    def box_confirm_cb(self,req):
        res = box_confirmResponse()
        self.pallet_handler.confirmBoxRecovery(req.box_number)
        res.acknowledged = True
        return res
        
