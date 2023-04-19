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
import pickle

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import scoUtil
from abc import abstractmethod

#import skeletonLabeling
#import skeletonLabelingHepatic

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


class CAPPRegistration :
    s_cameraDist = 300

    eAorta = 0
    eCT = 1
    eVA = 2
    eTotal = 3
    # key : enumIndex
    # value : list
    #           0 : nifti fileName
    #           1 : info mask  
    #           2 : info name
    #'''
    s_dicInfo = {
        eAorta : ["Aorta.nii.gz", 1, "Aorta"],
        eCT : ["CT.nii.gz", 2, "CT"],
        eVA : ["VA.nii.gz", 3, "VA"],
    }
    s_listStrTmp = ["eapReg", "eapReg", "pp"]
    #s_listStrTmp = ["eap", "eap", "pp"]
    #'''


    @staticmethod
    def get_info_file_name(eInfo : int) :
        return CAPPRegistration.s_dicInfo[eInfo][0]
    @staticmethod
    def get_info_mask(eInfo : int) :
        return CAPPRegistration.s_dicInfo[eInfo][1]
    @staticmethod
    def get_info_name(eInfo : int) :
        return CAPPRegistration.s_dicInfo[eInfo][2]
    @staticmethod
    def get_info_key(eInfo : int) :
        return f"key_{CAPPRegistration.get_info_name(eInfo)}"
    

    def __init__(self, title : str, width : int, height : int, dataPath : str) -> None:
        self.m_list_cb_callback = [
            self._on_cb_aorta,
            self._on_cb_ct,
            self._on_cb_va,
        ]
        self.m_listCB = []
        self.m_dicInfo = {}

        self.m_title = title
        self.m_width = width
        self.m_height = height
        self.m_dataPath = dataPath

        self.m_app = gui.Application.instance
        self.m_app.initialize()
        self.m_win = self.m_app.create_window(title, width, height)

        self.init_mtrl()
        self.init_scene()

        self.m_mainLayout = gui.Vert()
        self.m_mainLayout.add_child(self.init_selection_layout())
        self.m_mainLayout.add_fixed(30)
        self.m_mainLayout.add_child(gui.Label("Data List"))
        for inx in range(0, self.eTotal) :
            self.m_mainLayout.add_child(self.init_data_list_layout(inx))

        self.m_win.add_child(self.m_widget)
        self.m_win.add_child(self.m_mainLayout)
        self.m_win.set_on_layout(self._on_layout)

        self.state_init()
    def init_mtrl(self) :
        self.m_mtrlEAP = rendering.MaterialRecord()
        self.m_mtrlEAP.shader = "defaultUnlit"  # to use line_width property
        self.m_mtrlEAP.base_color = (1, 0, 0, 1)
        self.m_mtrlEAP.point_size = 2

        self.m_mtrlPP = rendering.MaterialRecord()
        self.m_mtrlPP.shader = "defaultUnlit"  # to use line_width property
        self.m_mtrlPP.base_color = (0, 0, 1, 1)
        self.m_mtrlPP.point_size = 2

    def init_scene(self) :
        self.m_widget = gui.SceneWidget()
        # scene 구성
        self.m_widget.scene = rendering.Open3DScene(self.m_win.renderer)
    def init_selection_layout(self) :
        btnXView = gui.Button("XView")
        btnXView.set_on_clicked(self._on_btn_x_view)
        btnYView = gui.Button("YView")
        btnYView.set_on_clicked(self._on_btn_y_view)
        btnZView = gui.Button("ZView")
        btnZView.set_on_clicked(self._on_btn_z_view)

        cameraLayout = gui.Horiz()
        cameraLayout.add_child(btnXView)
        cameraLayout.add_child(btnYView)
        cameraLayout.add_child(btnZView)
        
        return cameraLayout
    def init_data_list_layout(self, eData : int) :
        cb = gui.Checkbox("")
        cb.set_on_checked(self.m_list_cb_callback[eData])
        labelOrgan = gui.Label(self.get_info_name(eData))
        self.m_listCB.append(cb)

        layoutInfo = gui.Horiz(0, gui.Margins(5, 5, 5, 5))
        layoutInfo.add_child(cb)
        layoutInfo.add_child(labelOrgan)

        return layoutInfo
    def clear(self) :
        self.m_widget.scene.clear_geometry()
        for cb in self.m_listCB :
            cb.checked = False

        # 추후에 3d_label도 remove 시켜야 함. 


    def process(self) :
        self.m_app.run()


    def load_info(self) :
        # checking validation arterial vessel
        '''
        for eInfo in range(0, self.eTotal) :
            fileName = self.get_info_file_name(eInfo)
            fullPath = os.path.join(self.m_dataPath, fileName)
            if not os.path.exists(fullPath) :
                print(f"none-exist arterial vessel : {fileName}")
        '''
        # loading data
        for eInfo in range(0, self.eTotal) :
            fileName = self.get_info_file_name(eInfo)
            fullPath = os.path.join(self.m_dataPath, self.s_listStrTmp[eInfo])
            fullPath = os.path.join(fullPath, fileName)

            sitk = scoUtil.CScoUtilSimpleITK.load_image(fullPath, None)
            npImg = scoUtil.CScoUtilSimpleITK.sitkImg_to_npImg(sitk, "uint8")
            listCoord = self.get_vessel_coord(npImg)
            pcd = scoUtil.CScoUtilSimpleITK.get_pcd_from_list(listCoord, (1, 1, 1))
            pcdAABB = scoUtil.CScoUtilSimpleITK.get_aabb_from_point_cloud(pcd, (1, 1, 1))
            self.m_dicInfo[self.get_info_key(eInfo)] = [listCoord, pcd, pcdAABB, sitk]

            scoUtil.CScoUtilSimpleITK.print_sitk_img_info(sitk)

    def update_scene(self) :
        # flag setting 
        #widget.scene.set_background((0, 0, 0, 1))
        self.m_widget.scene.show_axes(True)
        self.m_widget.scene.show_skybox(True)
        bbox = self.m_widget.scene.bounding_box
        # camera 
        self.m_widget.setup_camera(60.0, bbox, bbox.get_center())
    def update_scene_info(self, bAdded : bool, eInfo : int, mtrl) :
        key = self.get_info_key(eInfo)
        if bAdded == False :
            self.m_widget.scene.remove_geometry(key)
        else :
            self.m_widget.scene.add_geometry(key, self.m_dicInfo[key][1], mtrl)
    '''
    def update_scene_info(self, bAdded : bool, eInfo : int, mtrl, npMat : np.ndarray) :
        key = self.get_info_key(eInfo)
        if bAdded == False :
            self.m_widget.scene.remove_geometry(key)
        else :
            self.m_widget.scene.add_geometry(key, self.m_dicInfo[key][1], mtrl)
            self.m_widget.scene.set_geometry_transform(key, npMat)
    '''
    def update_coord(self, eInfo : int, npMat : np.ndarray) :
        key = self.get_info_key(eInfo)
        if self.m_widget.scene.has_geometry(key) == True :
            self.m_widget.scene.set_geometry_transform(key, npMat)
    def get_vessel_coord(self, npImg : np.ndarray) :
        tmpCoord = np.array(np.where(npImg > 0), dtype="uint32").T
        listCoord = [coord for coord in tmpCoord]
        return listCoord
    def get_organ_coord(self, npImg : np.ndarray, iSpace = 20) :
        tmpCoord = np.array(np.where(npImg > 0), dtype="uint32").T
        listCoord = [coord for inx, coord in enumerate(tmpCoord) if inx % iSpace == 0]
        return listCoord
    
    # ui event
    def _on_layout(self, layout_context):
        r = self.m_win.content_rect

        rectWidget = gui.Rect()
        rectWidget.x = 0
        rectWidget.y = 0
        rectWidget.width = int(r.width * 0.7 + 0.5)
        rectWidget.height = r.height
        self.m_widget.frame = rectWidget

        rectLayOut = gui.Rect()
        rectLayOut.x = rectWidget.width
        rectLayOut.y = 0
        rectLayOut.width = int(r.width * 0.3 + 0.5)
        rectLayOut.height = rectWidget.height
        self.m_mainLayout.frame = rectLayOut

    def _on_btn_x_view(self) :
        bbox = self.m_widget.scene.bounding_box
        boxCenter = bbox.get_center()
        camPosXView = scoUtil.CScoMath.vec_add(boxCenter, np.array([self.s_cameraDist, 0, 0]))
        self.m_widget.scene.camera.look_at(boxCenter, camPosXView, [0, 1, 0])
    def _on_btn_y_view(self) :
        bbox = self.m_widget.scene.bounding_box
        boxCenter = bbox.get_center()
        camPosYView = scoUtil.CScoMath.vec_add(boxCenter, np.array([0, self.s_cameraDist, 0]))
        self.m_widget.scene.camera.look_at(boxCenter, camPosYView, [0, 1, 0])
    def _on_btn_z_view(self) :
        bbox = self.m_widget.scene.bounding_box
        boxCenter = bbox.get_center()
        camPosZView = scoUtil.CScoMath.vec_add(boxCenter, np.array([0, 0, self.s_cameraDist]))
        self.m_widget.scene.camera.look_at(boxCenter, camPosZView, [0, 1, 0])

    def _on_cb_aorta(self, bChecked) :
        self.update_scene_info(bChecked, self.eAorta, self.m_mtrlEAP)
    def _on_cb_ct(self, bChecked):
        self.update_scene_info(bChecked, self.eCT, self.m_mtrlEAP)
    def _on_cb_va(self, bChecked):
        self.update_scene_info(bChecked, self.eVA, self.m_mtrlPP)


    def state_init(self) :
        self.clear()
        self.load_info()

        self.m_win.title = self.m_title

        # construction scene
        self.m_listCB[self.eAorta].checked = True
        self._on_cb_aorta(True)
        self.update_scene()

if __name__ == '__main__' :
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_9"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10"
    dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_11"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_12"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_13"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_14"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_15"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_16"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_17"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_18"
    app = CAPPRegistration(dataPath, 1280, 720, dataPath)
    app.process()


print("clear ~~")

