#!/usr/bin/env python
# coding=utf-8
from pallet_database import PalletDatabase


if __name__ == "__main__":
    pallet_path = "/home/davide/ros/coorsa_ws/src/pallet_database_pkg/data/pallet_data.xml"
    database = PalletDatabase(pallet_path)