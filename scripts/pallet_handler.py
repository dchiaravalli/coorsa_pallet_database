#!/usr/bin/env python
# coding=utf-8
import xml.etree.ElementTree as ET
from pallet import Pallet

class PalletHandler:
    
    def __init__(self,pallet_file_path):
        self.pallet_file_path = pallet_file_path
        self.pallet_map = {}
        self.pallet_dict = {}
        self.current_pallet_name = ''

        self.getPalletData()                                #read pallet data from xml and create data dictionary
        self.createPalletMap()                              #list all pallets according to box type
        self.orderPalletMap()                               #set picking ordering on pallets of the same box type

    def getPalletData(self):
        tree = ET.parse(self.pallet_file_path)
        root = tree.getroot()
        layers = 0

        for code in root:
            for subitem in code.findall('subitem'):
                if subitem.get('name') == 'box':
                    for dat in subitem.findall('dat'):
                        if dat.get('name') == 'dim_x':
                            box_x = float(dat.text)
                        if dat.get('name') == 'dim_y':
                            box_y = float(dat.text)
                        if dat.get('name') == 'dim_z':
                            box_z = float(dat.text)
                        if dat.get('name') == 'type':
                            box_t = float(dat.text)
                if subitem.get('name') == 'pallet':
                    for dat in subitem.findall('dat'):
                        if dat.get('name') == 'dim_x':
                            dim_x = float(dat.text)
                        if dat.get('name') == 'dim_y':
                            dim_y = float(dat.text)
                        if dat.get('name') == 'dim_z':
                            dim_z = float(dat.text)
                        if dat.get('name') == 'pos_x':
                            pos_x = float(dat.text)
                        if dat.get('name') == 'pos_y':
                            pos_y = float(dat.text)
                        if dat.get('name') == 'pos_z':
                            pos_z = float(dat.text)
                        if dat.get('name') == 'layers':
                            layers = float(dat.text)
            self.pallet_dict[code.get('name')] = Pallet([box_x,box_y,box_z],box_t,[pos_x,pos_y,pos_z],[dim_x,dim_y,dim_z],layers)
        print(self.pallet_dict)

    def createPalletMap(self):                                              #list all pallets according to box type
        for item in self.pallet_dict.keys():
            item_type = self.pallet_dict[item].getBoxType()
            if item_type not in self.pallet_map.keys():
                self.pallet_map[item_type] = []
            self.pallet_map[item_type].append(item)

    def orderPalletMap(self):
        pass
            
    def getPalletPose(self, box_type = 1):
        self.current_pallet_name = self.pallet_map[box_type][0]
        pallet_data = {}
        pallet_data['name'] = self.current_pallet_name
        pallet_data['pose'] = self.pallet_dict[self.current_pallet_name].getPalletPose()
        return pallet_data

    def getBoxDimensions(self, box_type = 1):
        self.current_pallet_name = self.pallet_map[box_type][0]
        pallet_data = {}
        pallet_data['name'] = self.current_pallet_name
        pallet_data['box_dim'] = self.pallet_dict[self.current_pallet_name].getBoxDimensions()
        return pallet_data

    def getPalletInfo(self, box_type = 1):
        self.current_pallet_name = self.pallet_map[box_type][0]
        pallet_data = {}
        pallet_data['name'] = self.current_pallet_name
        pallet_data['pose'] = self.pallet_dict[self.current_pallet_name].getPalletPose()
        pallet_data['dim'] = self.pallet_dict[self.current_pallet_name].getPalletDim()
        pallet_data['box_dim'] = self.pallet_dict[self.current_pallet_name].getBoxDimensions()
        if not self.pallet_dict[self.current_pallet_name].box_list:
            pallet_data['box_detected'] = False
        else:
            pallet_data['box_detected'] = True
        return pallet_data

    def setPalletDetectionData(self, pallet_data):
        self.pallet_dict[pallet_data['name']].setPalletPickingPlane(pallet_data['plane'])
        self.pallet_dict[pallet_data['name']].setBoxesOnPlane(pallet_data['box_list'])

    def getBoxPoses(self, box_number):
        return self.pallet_dict[self.current_pallet_name].queryBoxRecoveryPose(box_number)

    def confirmBoxRecovery(self, box_number):
        return self.pallet_dict[self.current_pallet_name].confirmBoxRecovery(box_number)
