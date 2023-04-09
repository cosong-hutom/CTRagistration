import numpy as np
import matplotlib.pyplot as plt
import SimpleITK as sitk
import cv2
import os, sys
import open3d as o3d
import open3d.core
import open3d.visualization
import matplotlib.patches as patches
#from matplotlib.patches import Rectangle
#import pickle

#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import scoUtil
import scoReg

import argparse as ap

if __name__ == '__main__' :
    #'''
    parser = ap.ArgumentParser()
    parser.add_argument('-eap', type=str, dest='eapPath') 
    parser.add_argument('-pp', type=str, dest='ppPath')
    parser.add_argument('-o', type=str, dest='outEapPath')
    args = parser.parse_args()

    eapPath = args.eapPath
    ppPath = args.ppPath
    outEapPath = args.outEapPath
    #'''

    '''
    eapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/eap"
    ppPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/pp"
    outEapPath="/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10/eapReg"
    '''

    engineReg = scoReg.CRegistrationCT(eapPath, ppPath)
    if engineReg.Ready == False :
        print("failed initializing Registration")
        exit()
        
    engineReg.load_info()
    engineReg.process()
    
    print(f"offset : {engineReg.OffsetX}, {engineReg.OffsetY}, {engineReg.OffsetZ}")

    scoUtil.CScoUtilOS.create_directory(outEapPath)
    listFile = os.listdir(eapPath)

    for file in listFile :
        fileName, fileExt = os.path.splitext(file)

        if fileExt != ".gz" :
            continue

        inFullPath = os.path.join(eapPath, file)
        outFullPath = os.path.join(outEapPath, file)
        engineReg.process_resample(inFullPath, outFullPath)

        print(f"complete coping {outFullPath}")






