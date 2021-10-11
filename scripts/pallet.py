#!/usr/bin/env python
# coding=utf-8
class Pallet:

    def __init__(self, box_dimensions, box_type, pallet_pose, pallet_dim, pallet_layers, approaching_pose):
        self.box_dimensions = box_dimensions        #dimension of pallet boxes on order length, width, height
        self.box_type = box_type                    #type of box (some tag used by database to sort by type)
        self.pallet_pose = pallet_pose              #position of the center of the pallet w.r.t. world reference frame
        self.pallet_dim = pallet_dim                #dimension of the pallet on order length width height (0 if not known)
        self.pallet_layers = pallet_layers
        self.approaching_pose = approaching_pose
        self.box_list = []
        self.layer_dimensions = [0,0,0]

    def queryBoxRecoveryPose(self, num_boxes):              #ask for pose of the next boxes to be collected from the pallet
        if not self.box_list:
            box_recovery_list=[]
            for i in range(num_boxes):
                box_recovery_list.append(self.box_list[i])
        return box_recovery_list

    def confirmBoxRecovery(self, num_boxes):                #confirmation of success recovery ->box removed from list
        for i in range(num_boxes):
            self.box_list.remove(0)


    ###get variable functions
    def getPalletPose(self):                                #get position of center of pallet
        return self.pallet_pose

    def getPalletDim(self):                                 #get dimensions of pallet
        return self.pallet_dim

    def getBoxDimensions(self):                             #get dimenions of boxes
        return self.box_dimensions

    def getBoxType(self):                                   #get type of boxes
        return self.box_type

    def getApproachingPose(self):
        return self.approaching_pose

    def getPlaneEstimate(self):                             #get estimate of next plane
        #TODO
        pass


    ###set variable functions
    def setPalletPickingPlane(self, plane):                 #set plane parameters for box detection
        self.plane = plane                                  #parameters a,b,c,d describing the plane w.r.t. the world reference frame

    def setBoxesOnPlane(self, box_list):                    #set list of detected boxes for picking
        self.box_list = box_list

    def setLayerDimensions(self, layer_dimensions):         #set dimension of layers (interfalda)
        self.layer_dimensions = layer_dimensions
