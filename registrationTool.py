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
#import scoData
#from abc import abstractmethod

#import skeletonLabeling
#import skeletonLabelingHepatic

import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

#from skimage.morphology import skeletonize
#from skimage.morphology import skeletonize, skeletonize_3d
#from skimage import data
#from skimage.util import invert






class CAPPRegistration :
    s_cameraDist = 300
    s_eapPath = "eap"
    #s_eapPath = "eapReg"
    s_ppPath = "pp"

    def __init__(self, title : str, width : int, height : int, dataPath : str) -> None:
        self.m_bExit = False
        self.m_title = title
        self.m_width = width
        self.m_height = height
        self.m_dataPath = dataPath
        self.m_engineReg = None

        self.m_app = gui.Application.instance
        self.m_app.initialize()
        self.m_win = self.m_app.create_window(title, width, height)

        self.init_mtrl()
        self.init_scene()

        self.m_mainLayout = gui.Vert()
        self.m_mainLayout.add_child(self.init_selection_layout())
        self.m_mainLayout.add_fixed(30)
        self.m_mainLayout.add_child(gui.Label("Offset Info"))
        self.m_mainLayout.add_child(self.init_offset_layout())

        self.m_mainLayout.add_fixed(30)
        self.m_mainLayout.add_child(gui.Label("Dice Score Info"))
        self.m_mainLayout.add_child(self.init_dice_layout())

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
        self.m_widget.set_on_key(self._on_key)
        #self.m_widget.set_on_mouse(self._on_mouse)
    def init_selection_layout(self) :
        btnXView = gui.Button("XView")
        btnXView.set_on_clicked(self._on_btn_x_view)
        btnYView = gui.Button("YView")
        btnYView.set_on_clicked(self._on_btn_y_view)
        btnZView = gui.Button("ZView")
        btnZView.set_on_clicked(self._on_btn_z_view)
        btnReset = gui.Button("Reset")
        btnReset.set_on_clicked(self._on_btn_reset)

        cameraLayout = gui.Horiz()
        cameraLayout.add_child(btnXView)
        cameraLayout.add_child(btnYView)
        cameraLayout.add_child(btnZView)
        cameraLayout.add_child(btnReset)

        vert = gui.Vert()
        vert.add_child(cameraLayout)
        
        return vert
    def init_offset_layout(self) :
        listLabel = []
        self.m_listEditOffset = []
        listBtnDec = []
        listBtnInc = []

        listLabel.append(gui.Label("Offset X : "))
        listLabel.append(gui.Label("Offset Y : "))
        listLabel.append(gui.Label("Offset Z : "))
        self.m_listEditOffset.append(gui.TextEdit())
        self.m_listEditOffset.append(gui.TextEdit())
        self.m_listEditOffset.append(gui.TextEdit())
        listBtnDec.append(gui.Button("X--"))
        listBtnDec.append(gui.Button("Y--"))
        listBtnDec.append(gui.Button("Z--"))
        listBtnInc.append(gui.Button("X++"))
        listBtnInc.append(gui.Button("Y++"))
        listBtnInc.append(gui.Button("Z++"))

        listBtnDec[0].set_on_clicked(self._on_btn_x_dec)
        listBtnInc[0].set_on_clicked(self._on_btn_x_inc)
        listBtnDec[1].set_on_clicked(self._on_btn_y_dec)
        listBtnInc[1].set_on_clicked(self._on_btn_y_inc)
        listBtnDec[2].set_on_clicked(self._on_btn_z_dec)
        listBtnInc[2].set_on_clicked(self._on_btn_z_inc)

        vert = gui.Vert()

        for inx, label in enumerate(listLabel) :
            self.m_listEditOffset[inx].text_value = "0"
            horiz = gui.Horiz(0, gui.Margins(2, 1, 100, 1))  # left, bottom, right, top
            horiz.add_child(listBtnDec[inx])
            horiz.add_child(listBtnInc[inx])
            horiz.add_child(listLabel[inx])
            horiz.add_child(self.m_listEditOffset[inx])
            vert.add_child(horiz)
        
        return vert
    def init_dice_layout(self) :
        label = gui.Label("Dice : ")
        self.m_editDice = gui.TextEdit()

        horiz = gui.Horiz(0, gui.Margins(2, 1, 200, 1))  # left, bottom, right, top
        horiz.add_child(label)
        horiz.add_child(self.m_editDice)

        btnAutoReg = gui.Button("Auto Registration")
        btnAutoReg.set_on_clicked(self._on_btn_auto_registration)

        vert = gui.Vert()
        vert.add_child(horiz)
        vert.add_child(btnAutoReg)

        return vert
    def clear(self) :
        self.m_widget.scene.clear_geometry()
        if self.m_engineReg is not None :
            self.m_engineReg.clear()
        self.m_engineReg = None

        # 추후에 3d_label도 remove 시켜야 함. 


    def process(self) :
        #self.m_app.run()

        while self.m_app.run_one_tick() == True :
            if self.m_bExit == True :
                self.m_app.quit()

            # rendering loop
        
        self.clear()
        

    def update_scene(self) :
        # flag setting 
        #widget.scene.set_background((0, 0, 0, 1))
        self.m_widget.scene.show_axes(True)
        self.m_widget.scene.show_skybox(True)
        bbox = self.m_widget.scene.bounding_box
        # camera 
        self.m_widget.setup_camera(60.0, bbox, bbox.get_center())
    '''
    def update_scene_info(self, bAdded : bool, eInfo : int, mtrl) :
        key = self.get_info_key(eInfo)
        if bAdded == False :
            self.m_widget.scene.remove_geometry(key)
        else :
            self.m_widget.scene.add_geometry(key, self.m_dicInfo[key][1], mtrl)
    '''
    def update_scene_info(self, bAdded : bool, eInfo : int, mtrl, npMat : np.ndarray) :
        key = self.m_engineReg.get_info_key(eInfo)
        if bAdded == False :
            self.m_widget.scene.remove_geometry(key)
        else :
            self.m_widget.scene.add_geometry(key, self.m_engineReg.get_pcd(eInfo), mtrl)
            self.m_widget.scene.set_geometry_transform(key, npMat)
    def update_scene_info_aabb(self, bAdded : bool, eInfo : int, mtrl, npMat : np.ndarray) :
        key = self.m_engineReg.get_info_key(eInfo)
        keyAABB = self.m_engineReg.get_info_aabb_key(eInfo)
        if bAdded == False :
            self.m_widget.scene.remove_geometry(keyAABB)
        else :
            self.m_widget.scene.add_geometry(keyAABB, self.m_engineReg.get_pcd_aabb(eInfo), mtrl)
            self.m_widget.scene.set_geometry_transform(keyAABB, npMat)
    def update_coord(self, eInfo : int, npMat : np.ndarray) :
        key = self.m_engineReg.get_info_key(eInfo)
        if self.m_widget.scene.has_geometry(key) == True :
            self.m_widget.scene.set_geometry_transform(key, npMat)
    
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
    def _on_btn_reset(self) :
        self.m_listEditOffset[0].text_value = f"{0}"
        self.m_listEditOffset[1].text_value = f"{0}"
        self.m_listEditOffset[2].text_value = f"{0}"
        self.state_exe()
    def _on_btn_x_dec(self) :
        offsetX = int(self.m_listEditOffset[0].text_value)
        offsetX -= 1
        self.m_listEditOffset[0].text_value = f"{offsetX}"
        self.state_exe()
    def _on_btn_x_inc(self) :
        offsetX = int(self.m_listEditOffset[0].text_value)
        offsetX += 1
        self.m_listEditOffset[0].text_value = f"{offsetX}"
        self.state_exe()
    def _on_btn_y_dec(self) :
        offsetY = int(self.m_listEditOffset[1].text_value)
        offsetY -= 1
        self.m_listEditOffset[1].text_value = f"{offsetY}"
        self.state_exe()
    def _on_btn_y_inc(self) :
        offsetY = int(self.m_listEditOffset[1].text_value)
        offsetY += 1
        self.m_listEditOffset[1].text_value = f"{offsetY}"
        self.state_exe()
    def _on_btn_z_dec(self) :
        offsetZ = int(self.m_listEditOffset[2].text_value)
        offsetZ -= 1
        self.m_listEditOffset[2].text_value = f"{offsetZ}"
        self.state_exe()
    def _on_btn_z_inc(self) :
        offsetZ = int(self.m_listEditOffset[2].text_value)
        offsetZ += 1
        self.m_listEditOffset[2].text_value = f"{offsetZ}"
        self.state_exe()
    def _on_btn_auto_registration(self) :
        self.m_engineReg.process()

        print("clear auto registration")

        diceScore = self.m_engineReg.DiceScore
        offsetX = self.m_engineReg.OffsetX
        offsetY = self.m_engineReg.OffsetY
        offsetZ = self.m_engineReg.OffsetZ

        self.m_editDice.text_value = str(diceScore)
        self.m_listEditOffset[0].text_value = str(offsetX)
        self.m_listEditOffset[1].text_value = str(offsetY)
        self.m_listEditOffset[2].text_value = str(offsetZ)

        npMatEAPToPP = self.m_engineReg.NowMatEAPToPP
        self.update_scene_info(True, self.m_engineReg.eCT, self.m_mtrlEAP, npMatEAPToPP)
        self.update_scene_info_aabb(True, self.m_engineReg.eCT, self.m_mtrlEAP, npMatEAPToPP)

    def _on_key(self, e):
        '''
        if e.key == gui.KeyName.SPACE:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                print("[debug] SPACE released")
            else:
                print("[debug] SPACE pressed")
            return gui.Widget.EventCallbackResult.HANDLED
        if e.key == gui.KeyName.W:  # eats W, which is forward in fly mode
            print("[debug] Eating W")
            return gui.Widget.EventCallbackResult.CONSUMED
        '''
        
        if e.key == gui.KeyName.Q :
            print("pressed Q")
            self.m_bExit = True
            return gui.Widget.EventCallbackResult.CONSUMED
        
        return gui.Widget.EventCallbackResult.IGNORED
    def _on_mouse(e):
        if e.type == gui.MouseEvent.Type.BUTTON_DOWN:
            print("[debug] mouse:", (e.x, e.y))
        return gui.Widget.EventCallbackResult.IGNORED

    def state_init(self) :
        #self.clear()

        self.m_win.title = self.m_title
        eapPath = os.path.join(self.m_dataPath, self.s_eapPath)
        ppPath = os.path.join(self.m_dataPath, self.s_ppPath)

        # checking validation arterial vessel
        '''
        for eInfo in range(0, self.eTotal) :
            fileName = self.get_info_file_name(eInfo)
            fullPath = os.path.join(self.m_dataPath, fileName)
            if not os.path.exists(fullPath) :
                print(f"not found file : {fileName}")
                return
        '''

        self.m_engineReg = scoReg.CRegistrationCT(eapPath, ppPath)
        if self.m_engineReg.Ready == False :
            print("failed initializing Registration")
            return
        
        self.m_engineReg.load_info()
        self.m_engineReg.process_with_offset(0, 0, 0)
        diceScore = self.m_engineReg.DiceScore
        self.m_editDice.text_value = str(diceScore)

        # construction scene
        self.update_scene_info(True, self.m_engineReg.eCT, self.m_mtrlEAP, self.m_engineReg.MatEAPToPP)
        self.update_scene_info_aabb(True, self.m_engineReg.eCT, self.m_mtrlEAP, self.m_engineReg.MatEAPToPP)
        self.update_scene_info(True, self.m_engineReg.eVA, self.m_mtrlPP, self.m_engineReg.MatPP)
        self.update_scene()
    def state_exe(self) :
        offsetX = int(self.m_listEditOffset[0].text_value)
        offsetY = int(self.m_listEditOffset[1].text_value)
        offsetZ = int(self.m_listEditOffset[2].text_value)

        self.m_engineReg.process_with_offset(offsetX, offsetY, offsetZ)
        diceScore = self.m_engineReg.DiceScore
        self.m_editDice.text_value = str(diceScore)

        npMatEAPToPP = self.m_engineReg.NowMatEAPToPP
        self.update_scene_info(True, self.m_engineReg.eCT, self.m_mtrlEAP, npMatEAPToPP)
        self.update_scene_info_aabb(True, self.m_engineReg.eCT, self.m_mtrlEAP, npMatEAPToPP)

        # dbg
        '''
        key = "key_dbg"
        npMat = scoUtil.CScoMath.make_mat4x4_translate_3d(128, 256, 256)
        if self.m_widget.scene.has_geometry(key) == True :
            self.m_widget.scene.remove_geometry(key)
        
        self.m_widget.scene.add_geometry(key, self.dbg_pcd, self.m_mtrlPP)
        self.m_widget.scene.set_geometry_transform(key, npMat)
        '''

if __name__ == '__main__' :
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_9"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_10"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_11"
    dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_12"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_13"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_14"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_15"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_16"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_17"
    #dataPath = "/Users/scosco/Desktop/scosco/hutom/Project/Registration/Project/Data/01009ug_18"
    app = CAPPRegistration(dataPath, 1280, 720, dataPath)
    app.process()


print("clear ~~")



