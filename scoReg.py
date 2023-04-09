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


class CRegistrationCT :
    #eAorta = 0
    eCT = 0
    eVA = 1
    eTotal = 2
    # key : enumIndex
    # value : list
    #           0 : nifti fileName
    #           1 : info mask  
    #           2 : info name
    #'''
    s_dicInfo = {
        #eAorta : ["Aorta.nii.gz", 1, "Aorta"],
        eCT : ["CT.nii.gz", 2, "CT"],
        eVA : ["VA.nii.gz", 3, "VA"],
    }
    #'''


    @staticmethod
    def get_info_file_name(eInfo : int) :
        return CRegistrationCT.s_dicInfo[eInfo][0]
    @staticmethod
    def get_info_mask(eInfo : int) :
        return CRegistrationCT.s_dicInfo[eInfo][1]
    @staticmethod
    def get_info_name(eInfo : int) :
        return CRegistrationCT.s_dicInfo[eInfo][2]
    @staticmethod
    def get_info_key(eInfo : int) :
        return f"key_{CRegistrationCT.get_info_name(eInfo)}"
    @staticmethod
    def get_info_aabb_key(eInfo : int) :
        return f"key_aabb_{CRegistrationCT.get_info_name(eInfo)}"
    
    
    def __init__(self, eapPath : str, ppPath : str) :
        self.m_eapPath = eapPath
        self.m_ppPath = ppPath
        self.m_bReady = False
            
        self.m_bReady = True
            
        self.m_dicInfo = {}
        self.m_npMatEAPToPP = scoUtil.CScoMath.make_mat4x4_identity()
        self.m_npMatPP = scoUtil.CScoMath.make_mat4x4_identity()

        self.m_outDiceScore = 0
        self.m_offsetX = 0
        self.m_offsetY = 0
        self.m_offsetZ = 0
        self.m_outNowMatEAPToPP = scoUtil.CScoMath.make_mat4x4_identity()
    def clear(self) :
        self.m_eapPath = ""
        self.m_ppPath = ""
        self.m_bReady = False
        self.m_dicInfo.clear()
        self.m_npMatEAPToPP = scoUtil.CScoMath.make_mat4x4_identity()
        self.m_npMatPP = scoUtil.CScoMath.make_mat4x4_identity()

        self.m_outDiceScore = 0
        self.m_offsetX = 0
        self.m_offsetY = 0
        self.m_offsetZ = 0
        self.m_outNowMatEAPToPP = scoUtil.CScoMath.make_mat4x4_identity()
    
    def process_with_offset(self, offsetX, offsetY ,offsetZ) :
        self.m_offsetX = offsetX
        self.m_offsetY = offsetY
        self.m_offsetZ = offsetZ

        self.m_outDiceScore = self.get_dice_score(offsetX, offsetY, offsetZ)
        self.m_outNowMatEAPToPP = self.get_mat_eap_to_pp(offsetX, offsetY, offsetZ)
    def process(self) :
        self.process_with_offset(0, 0, 0)

        maxOffsetX = self.m_offsetX
        maxOffsetY = self.m_offsetY
        maxOffsetZ = self.m_offsetZ
        maxDiceScore = self.DiceScore

        bUpdate = True
        iIterCnt = 0

        while bUpdate == True and iIterCnt < 1000 :
            anchorDiceScore = maxDiceScore
            anchorOffsetX = maxOffsetX
            anchorOffsetY = maxOffsetY
            anchorOffsetZ = maxOffsetZ
            bUpdate = False

            for offsetZ in range(-1, 2) :
                nowOffsetZ = anchorOffsetZ + offsetZ
                for offsetY in range(-1, 2) :
                    nowOffsetY = anchorOffsetY + offsetY
                    for offsetX in range(-1, 2) :
                        nowOffsetX = anchorOffsetX + offsetX
                        nowDiceScore = self.get_dice_score(nowOffsetX, nowOffsetY, nowOffsetZ)
                        if nowDiceScore > maxDiceScore :
                            if offsetX == 0 and offsetY == 0 and offsetZ == 0 :
                                continue
                            maxDiceScore = nowDiceScore
                            maxOffsetX = nowOffsetX
                            maxOffsetY = nowOffsetY
                            maxOffsetZ = nowOffsetZ

                            bUpdate = True
            iIterCnt += 1

        self.m_offsetX = maxOffsetX
        self.m_offsetY = maxOffsetY
        self.m_offsetZ = maxOffsetZ  
        self.m_outDiceScore = maxDiceScore
        self.m_outNowMatEAPToPP = self.get_mat_eap_to_pp(self.m_offsetX, self.m_offsetY, self.m_offsetZ)    
    def process_resample(self, eapFullPath : str, outEapFullPath : str) :
        if not os.path.exists(eapFullPath) :
            print(f"not found file : {eapFullPath}")
            return
        
        sitkEAP = scoUtil.CScoUtilSimpleITK.load_image(eapFullPath, None)

        sitkVA = self.get_sitk(self.eVA)
        origin = sitkVA.GetOrigin()
        direction = sitkVA.GetDirection()
        spacing = sitkVA.GetSpacing()
        size = sitkVA.GetSize()

        #unsigned int dimensions, VectorDouble offset=std::vector< double >(3,0.0)
        offset = (self.OffsetX * spacing[0], self.OffsetY * spacing[1], self.OffsetZ * spacing[2])
        transform = sitk.TranslationTransform(3, offset)
        transform = transform.GetInverse()

        sitkEAP = sitk.Resample(
                    sitkEAP,
                    size,
                    transform,
                    sitk.sitkNearestNeighbor,
                    origin,
                    spacing,
                    direction,
                    0,
                    sitkVA.GetPixelID(),
                )
        
        scoUtil.CScoUtilSimpleITK.save_nifti(outEapFullPath, sitkEAP)


    @property
    def Ready(self) :
        return self.m_bReady
    @property
    def DiceScore(self) :
        return self.m_outDiceScore
    @property
    def OffsetX(self) :
        return self.m_offsetX
    @property
    def OffsetY(self) :
        return self.m_offsetY
    @property
    def OffsetZ(self) :
        return self.m_offsetZ
    @property
    def MatEAPToPP(self) :
        return self.m_npMatEAPToPP
    @property
    def MatPP(self) :
        return self.m_npMatPP
    @property
    def NowMatEAPToPP(self) :
        return self.m_outNowMatEAPToPP
    

    # resource
    def get_pcd(self, eData : int) :
        key = self.get_info_key(eData)
        return self.m_dicInfo[key][1]
    def get_pcd_aabb(self, eData : int) :
        key = self.get_info_key(eData)
        return self.m_dicInfo[key][2]
    def get_sitk(self, eData : int) :
        key = self.get_info_key(eData)
        return self.m_dicInfo[key][3]


    def load_info(self) :
        fileName = self.get_info_file_name(self.eCT)
        fullPath = os.path.join(self.m_eapPath, fileName)
        sitkCT = scoUtil.CScoUtilSimpleITK.load_image(fullPath, None)

        fileName = self.get_info_file_name(self.eVA)
        fullPath = os.path.join(self.m_ppPath, fileName)
        sitkVA = scoUtil.CScoUtilSimpleITK.load_image(fullPath, None)
        origin = sitkVA.GetOrigin()
        direction = sitkVA.GetDirection()
        spacing = sitkVA.GetSpacing()
        size = sitkVA.GetSize()

        sitkCT = sitk.Resample(
                    sitkCT,
                    size,
                    sitk.Transform(),
                    sitk.sitkNearestNeighbor,
                    origin,
                    spacing,
                    direction,
                    0,
                    sitkVA.GetPixelID(),
                )
        
        listSitk = [sitkCT, sitkVA]
    
        # loading data
        for eInfo in range(0, self.eTotal) :
            sitkImg = listSitk[eInfo]
            npImg = scoUtil.CScoUtilSimpleITK.sitkImg_to_npImg(sitkImg, "uint8")
            print(scoUtil.CScoUtilSimpleITK.get_min_max(sitkImg))
            npImg[npImg > 0] = 1
            listCoord = self.get_vessel_coord(npImg)
            pcd = scoUtil.CScoUtilSimpleITK.get_pcd_from_list(listCoord, (1, 1, 1))
            pcdAABB = scoUtil.CScoUtilSimpleITK.get_aabb_from_point_cloud(pcd, (1, 1, 1))
            self.m_dicInfo[self.get_info_key(eInfo)] = [listCoord, pcd, pcdAABB, sitkImg, npImg]

            scoUtil.CScoUtilSimpleITK.print_sitk_img_info(sitkImg)
        
        self.m_npMatEAPToPP = self.get_mat_eap_to_pp(0, 0, 0)

        eapAABB = self.m_dicInfo[self.get_info_key(self.eCT)][2]
        vecMin = eapAABB.get_min_bound()
        vecMax = eapAABB.get_max_bound()
        vecMin = scoUtil.CScoMath.make_vec4(vecMin[0], vecMin[1], vecMin[2], 1.0)
        vecMax = scoUtil.CScoMath.make_vec4(vecMax[0], vecMax[1], vecMax[2], 1.0)
        self.m_EAPToPPMin = scoUtil.CScoMath.mul_mat4x4_vec4(self.m_npMatEAPToPP, vecMin)
        self.m_EAPToPPMax = scoUtil.CScoMath.mul_mat4x4_vec4(self.m_npMatEAPToPP, vecMax)
        self.m_npEAPCrop = self.get_crop(self.eCT, vecMin, vecMax)

        print(f"min : {vecMin}, max : {vecMax}")
        print(f"pp_min : {self.m_EAPToPPMin}, pp_max : {self.m_EAPToPPMax}")
    def get_vessel_coord(self, npImg : np.ndarray) :
        tmpCoord = np.array(np.where(npImg > 0), dtype="uint32").T
        listCoord = [coord for coord in tmpCoord]
        return listCoord
    def get_organ_coord(self, npImg : np.ndarray, iSpace = 20) :
        tmpCoord = np.array(np.where(npImg > 0), dtype="uint32").T
        listCoord = [coord for inx, coord in enumerate(tmpCoord) if inx % iSpace == 0]
        return listCoord
    def get_mat_phy(self, eData : int, offsetX, offsetY, offsetZ) :
        sitk = self.m_dicInfo[self.get_info_key(eData)][3]
        origin = sitk.GetOrigin()
        direction = sitk.GetDirection()
        spacing = sitk.GetSpacing()
        size = sitk.GetSize()

        '''
        totalLen = np.array(size) * np.array(spacing) 
        center = np.array(size) * np.array(spacing) * 0.5

        print("*"*30)
        print(f"origin:{origin}")
        print(f"direction:{direction}")
        print(f"spacing:{spacing}")
        print(f"totalLen:{totalLen}")
        print(f"center:{center}")
        print("*"*30)
        '''

        offsetX = offsetX * spacing[0]
        offsetY = offsetY * spacing[1]
        offsetZ = offsetZ * spacing[2]

        npMatScale = scoUtil.CScoMath.make_mat4x4_scale(spacing[0], spacing[1], spacing[2])
        npMatRot = scoUtil.CScoMath.make_mat4x4_rot_from_column(direction)
        npMatTrans = scoUtil.CScoMath.make_mat4x4_translate_3d(origin[0] + offsetX, origin[1] + offsetY, origin[2] + offsetZ)

        npMatRet = scoUtil.CScoMath.mul_mat4x4(npMatTrans, npMatRot)

        return scoUtil.CScoMath.mul_mat4x4(npMatRet, npMatScale)
    def get_mat_eap_to_pp(self, offsetX, offsetY, offsetZ) :
        npMatEAPPhy = self.get_mat_phy(self.eCT, offsetX, offsetY, offsetZ)
        npMatPPPhy = self.get_mat_phy(self.eVA, 0, 0, 0)
        npMatPPPhyInv = scoUtil.CScoMath.make_mat4x4_inverse(npMatPPPhy)
        
        return scoUtil.CScoMath.mul_mat4x4(npMatPPPhyInv, npMatEAPPhy)
    def get_crop(self, eData : int, vecMin : np.ndarray, vecMax : np.ndarray) :
        npImg = self.m_dicInfo[self.get_info_key(eData)][4]
        iMin = (int(vecMin[0, 2] + 0.5), int(vecMin[0, 1] + 0.5), int(vecMin[0, 0] + 0.5))
        iMax = (int(vecMax[0, 2] + 0.5), int(vecMax[0, 1] + 0.5), int(vecMax[0, 0] + 0.5))

        return npImg[iMin[0] : iMax[0] + 1, iMin[1] : iMax[1] + 1, iMin[2] : iMax[2] + 1]
    def get_dice_score(self, offsetX, offsetY, offsetZ) :
        vecMin = self.m_EAPToPPMin + scoUtil.CScoMath.make_vec4(offsetX, offsetY, offsetZ, 1)
        vecMax = self.m_EAPToPPMax + scoUtil.CScoMath.make_vec4(offsetX, offsetY, offsetZ, 1)
        npPPCrop = self.get_crop(self.eVA, vecMin, vecMax)

        #listCoord = self.get_vessel_coord(npPPCrop)
        #self.dbg_pcd = scoUtil.CScoUtilSimpleITK.get_pcd_from_list(listCoord, (1, 1, 1))

        intersect = np.sum(npPPCrop * self.m_npEAPCrop)
        fsum = np.sum(npPPCrop)
        ssum = np.sum(self.m_npEAPCrop)
        dice = (2 * intersect ) / (fsum + ssum)
        #dice = np.mean(dice) # 이 부분은 빠져도 될 것 같은데 
        dice = round(dice, 3) # for easy reading

        return dice