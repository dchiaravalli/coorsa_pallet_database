#!/usr/bin/env python
# coding=utf-8
from pallet_database import PalletDatabase
import rospkg

if __name__ == "__main__":
    #"/home/davide/ros/coorsa_ws/src/pallet_database_pkg/data/pallet_data.xml"
	rospack = rospkg.RosPack()
	rospack.list()
	#pallet_path = rospack.get_path('coorsa_pallet_database')
	#pallet_path = pallet_path + '/data/pallet_data.xml'
	pallet_path = '/home/j/make_ros/src/coorsa_pallet_database/data/pallet_data.xml'
	database = PalletDatabase(pallet_path)
