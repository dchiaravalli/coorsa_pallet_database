#!/usr/bin/env python
# coding=utf-8
import rospy
import rospkg
import math
import copy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

waitingNewPoint = False
PointsClicked = []
PalletCorner = []
ApproachingPoint = []

def clickedOnPoint_callback(clk_point):
    global waitingNewPoint
    global PointsClicked
    waitingNewPoint = False
    PointsClicked.append(clk_point.point)

def DrawThePallet():
    global waitingNewPoint

    sub = rospy.Subscriber("clicked_point",PointStamped,clickedOnPoint_callback)

    rospy.logwarn("\nFollow these rules:\n1)\tThe first and the second point must define one of the approaccing sides.\n2)\tDefine it in a 'counterclockwise' way.\n3)\tThe third point must be a point of the opposite edge.")
    ######
    rospy.loginfo("Select 'Publish point' on RViz, then select the first corner")
    waitingNewPoint = True
    while(waitingNewPoint): pass
    ######
    rospy.loginfo("Select 'Publish point' on RViz, then select the second corner")
    waitingNewPoint = True
    while(waitingNewPoint): pass
    ######
    rospy.loginfo("Select 'Publish point' on RViz, then select the third corner\n")
    waitingNewPoint = True
    while(waitingNewPoint): pass
    ######
    sub.unregister()

def CreatePallet():
    global PointsClicked
    global PalletCorner

    a = PointsClicked[0]
    b = PointsClicked[1]
    c = PointsClicked[2]
    d = copy.deepcopy(c)

    #Effettuo i calcoli del '3-point rectangle':
    distance = abs((b.x - a.x)*(a.y - c.y) - (a.x - c.x)*(b.y - a.y))/math.sqrt(pow(b.x-a.x,2) + pow(b.y - a.y,2))
    #Vettori dal primo al secondo punto
    ABx = b.x - a.x
    ABy = b.y - a.y
    #Vettore perpendicolare
    PERPx = -ABy
    PERPy = ABx
    PERPSize = math.sqrt(ABx * ABx + ABy * ABy)
    #Versore perpendicolare
    UNITx = PERPx / PERPSize
    UNITy = PERPy / PERPSize
    #Vettore perpendicolare lungo quanto la distanza della retta (a,b) col punto c
    SVx = UNITx * distance
    SVy = UNITy * distance
    #Popolo l'array con i vertici così calcolati
    c.x = b.x + SVx
    c.y = b.y + SVy
    c.z = 0

    d.x = a.x + SVx
    d.y = a.y + SVy
    d.z = 0

    PalletCorner.append(a)
    PalletCorner.append(b)
    PalletCorner.append(c)
    PalletCorner.append(d)
    GenerateApproachingPoint(b,a,True)
    GenerateApproachingPoint(d,c,True)

    print("___ PALLET CREATED ___\n")

def PublishPalletPoligon():
    polygonPallet = PolygonStamped()
    pub = rospy.Publisher("visualization_polygon_pallet_1",PolygonStamped,queue_size=10)
    approach1 = rospy.Publisher("visualization_marker1",Marker,queue_size=10)
    approach2 = rospy.Publisher("visualization_marker2",Marker,queue_size=10)


    polygonPallet.header.frame_id = 'map';
    polygonPallet.polygon.points.append(PalletCorner[0])
    polygonPallet.polygon.points.append(PalletCorner[1])
    polygonPallet.polygon.points.append(PalletCorner[2])
    polygonPallet.polygon.points.append(PalletCorner[3])

    rate = rospy.Rate(10)

    #Pubblico un po' di messaggi per assicurarmi che
    #RViz li riceva
    pub.publish(polygonPallet)
    approach1.publish(ApproachingPoint[0])
    approach2.publish(ApproachingPoint[1])
    rate.sleep()
    pub.publish(polygonPallet)
    approach1.publish(ApproachingPoint[0])
    approach2.publish(ApproachingPoint[1])
    rate.sleep()
    pub.publish(polygonPallet)
    approach1.publish(ApproachingPoint[0])
    approach2.publish(ApproachingPoint[1])
    rate.sleep()
    pub.publish(polygonPallet)
    approach1.publish(ApproachingPoint[0])
    approach2.publish(ApproachingPoint[1])
    rate.sleep()

    #while not rospy.is_shutdown():
    #    pub.publish(polygonPallet)
    #    rate.sleep()

def prettify(element, indent='  '):
    queue = [(0, element)]  # (level, element)
    while queue:
        level, element = queue.pop(0)
        children = [(level + 1, child) for child in list(element)]
        if children:
            element.text = '\n' + indent * (level+1)  # for child open
        if queue:
            element.tail = '\n' + indent * queue[0][0]  # for sibling open
        else:
            element.tail = '\n' + indent * (level-1)  # for parent close
        queue[0:0] = children  #Format the xml

def EvaluatePalletPosition():
    ax=(PalletCorner[0].x+PalletCorner[1].x)/2
    ay=(PalletCorner[0].y+PalletCorner[1].y)/2
    az=(PalletCorner[0].z+PalletCorner[1].z)/2

    bx=(PalletCorner[2].x+PalletCorner[3].x)/2
    by=(PalletCorner[2].y+PalletCorner[3].y)/2
    bz=(PalletCorner[2].z+PalletCorner[3].z)/2

    centerX= (ax+bx)/2
    centerY= (ay+by)/2
    centerZ= (az+bz)/2

    center = copy.deepcopy(PalletCorner[0])
    center.x = centerX
    center.y = centerY
    center.z = centerZ
    center.x=round(center.x,2)
    center.y=round(center.y,2)
    center.z=round(center.z,2)
    return center

def EvaluatePalletDimension():
    dim = []
    dim.append(math.sqrt(pow(PalletCorner[1].x-PalletCorner[0].x,2)+pow(PalletCorner[1].y-PalletCorner[0].y,2)))
    dim.append(math.sqrt(pow(PalletCorner[2].x-PalletCorner[1].x,2)+pow(PalletCorner[2].y-PalletCorner[1].y,2)))
    dim.append(0)
    return dim

def SetBoxParameter(root):
    box_type = raw_input("Type a box type: ")
    box_info = [box_type,0,0,0]
    #Cerco tutti gli elementi 'subitem'
    boxes = root.findall(".//code/subitem[@name='box']")

    for b in boxes:
        type = b.find(".//dat[@name='type']")   #Prendo il campo Box e guardo se
        if(type!=None and type.text==box_type): #Coincide col tipo richiesto
            box_info[1] = b.find(".//dat[@name='dim_x']").text   #Se trovo una corrispondenza
            box_info[2] = b.find(".//dat[@name='dim_y']").text   #Ritorno le dimensioni di questa tipologia
            box_info[3] = b.find(".//dat[@name='dim_z']").text
            return box_info
    #Altrimenti definisco un nuovo tipo
    print("This is a new box type, please define its new dimensions:")
    box_info[1] = raw_input("\tdim_x: ")
    box_info[2] = raw_input("\tdim_y: ")
    box_info[3] = raw_input("\tdim_z: ")
    return box_info

def GenerateApproachingPoint(c1,c2,flip):
    global ApproachingPoint
    theta = 0;
    if((c2.x - c1.x) is not 0 ):
        theta = math.atan((c2.y - c1.y)/(c2.x - c1.x))
        if((c2.y > c1.y) and (c1.x > c2.x) or ((c2.y < c1.y) and (c2.x < c1.x)) ):
            theta += math.pi
    elif (c2.y < c1.y): theta = -math.pi/2
    elif (c2.y > c1.y): theta = math.pi/2

    theta = theta * 180/math.pi

    X1_ = 0
    X2_ = 0
    Y1_ = 0
    Y2_ = 0
    XA_ = 0
    YA_ = 0
    XA = 0
    YA = 0

    PERPx = 0;
    PERPy = 0;


    ABx = c2.x - c1.x
    ABy = c2.y - c1.y

    if(not flip):
        PERPx = ABy
        PERPy = -ABx
    else:
        PERPx = -ABy
        PERPy = ABx

    ###
    PERPSize = math.sqrt(ABx * ABx + ABy * ABy)
    ###
    UNITx = PERPx / PERPSize
    UNITy = PERPy / PERPSize
    ###
    SVx = UNITx * 0.5
    SVy = UNITy * 0.5
    ###
    XA = SVx + c1.x
    YA = SVy + c1.y

    _point = Marker()
    _point.pose.position.x = XA
    _point.pose.position.y = YA
    _point.pose.position.z = 0
    _point.pose.orientation.w = math.cos(theta*math.pi/360)
    _point.pose.orientation.z = math.sin(theta*math.pi/360)
    ##Per visualizzazione
    _point.header.frame_id = "map"
    _point.type = 0 #ARROW
    _point.action = 0 #ADD
    _point.id = 0
    _point.scale.x = 0.3
    _point.scale.y = 0.1
    _point.scale.z = 0.1
    _point.color.a = 1
    _point.color.r = 1
    _point.color.g = 0
    _point.color.b = 0
    ApproachingPoint.append(_point)



def SavePallet():
    global PointsClicked
    global PalletCorner
    global ApproachingPoint

    r = raw_input("Do you want to save this in pallet_data.xml? (yY/nN)")
    while(r!="y" and r!="Y" and r!="n" and r!="N"):
            r = raw_input("Do you want to save this in pallet_data.xml? (yY/nN)")

    if(r=="y" or r=="Y"):
        pkg_path = rospkg.RosPack().get_path("pallet_database_pkg")
        xml_doc = pkg_path + "/data/pallet_data.xml"
        tree = ET.parse(xml_doc)
        root = tree.getroot()

        pallet_name = raw_input("Type a name for the pallet: ")

        while(root.find(".//code[@name='" + pallet_name +"']") is not None):
            pallet_name = raw_input("A pallet named '" + pallet_name + "' already exists.\nType another name: ")

        code = ET.SubElement(root, 'code', name = pallet_name)
        box = ET.SubElement(code,'subitem', name = 'box')
        pallet = ET.SubElement(code,'subitem', name = 'pallet')
        approach = ET.SubElement(code,'subitem', name = 'approaching_poses')

        center=EvaluatePalletPosition()
        dimension=EvaluatePalletDimension()
        boxInfo=SetBoxParameter(root)

        #Subitem PALLET
        ET.SubElement(pallet,'dat',name = 'pos_x').text = str(center.x)
        ET.SubElement(pallet,'dat',name = 'pos_y').text = str(center.y)
        ET.SubElement(pallet,'dat',name = 'pos_z').text = str(center.z)
        ET.SubElement(pallet,'dat',name = 'dim_x').text = str(dimension[0])
        ET.SubElement(pallet,'dat',name = 'dim_y').text = str(dimension[1])
        ET.SubElement(pallet,'dat',name = 'dim_z').text = str(dimension[2])
        #Subitem BOX
        ET.SubElement(box,'dat',name = 'dim_x').text = str(boxInfo[1])
        ET.SubElement(box,'dat',name = 'dim_y').text = str(boxInfo[2])
        ET.SubElement(box,'dat',name = 'dim_z').text = str(boxInfo[3])
        ET.SubElement(box,'dat',name = 'type').text = str(boxInfo[0])
        #Subitem ApproachingPoses
        ET.SubElement(approach,'dat',name = 'pos_x1').text = str(ApproachingPoint[0].pose.position.x)
        ET.SubElement(approach,'dat',name = 'pos_y1').text = str(ApproachingPoint[0].pose.position.y)
        ET.SubElement(approach,'dat',name = 'or_w1').text = str(ApproachingPoint[0].pose.orientation.w)
        ET.SubElement(approach,'dat',name = 'or_z1').text = str(ApproachingPoint[0].pose.orientation.z)

        ET.SubElement(approach,'dat',name = 'pos_x2').text = str(ApproachingPoint[1].pose.position.x)
        ET.SubElement(approach,'dat',name = 'pos_y2').text = str(ApproachingPoint[1].pose.position.y)
        ET.SubElement(approach,'dat',name = 'or_w2').text = str(ApproachingPoint[1].pose.orientation.w)
        ET.SubElement(approach,'dat',name = 'or_z2').text = str(ApproachingPoint[1].pose.orientation.z)


        #Salvo il nuovo file xml
        root=tree.getroot()
        prettify(root)
        tree.write(xml_doc, encoding='UTF-8', xml_declaration=True)
        print("Pallet SAVED in: \n" + xml_doc + "\n")
    else:
        print("Try again...")
        PointsClicked = []
        PalletCorner = []
        ApproachingPoint = []
        DrawThePallet()
        CreatePallet()
        PublishPalletPoligon()
        SavePallet()



if __name__ == "__main__":
    rospy.init_node("PalletDrawer",anonymous = True)
    DrawThePallet()
    CreatePallet()
    PublishPalletPoligon()
    SavePallet()

    rospy.spin()
