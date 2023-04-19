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
import scoMath
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


class CRenderObj :
    def __init__(self) :
        self.m_key = ""

    
    @property
    def Key(self) :
        return self.m_key
    @Key.setter
    def Key(self, key : str) :
        self.m_key = key
class CRenderObjOBB(CRenderObj) :
    def __init__(self, key : str):
        super().__init__()

        # obb resource 
        listLine = [
            [0, 1], [1, 2], [2, 3], [3, 0],
            [4, 5], [5, 6], [6, 7], [7, 4],
            [0, 4], [1, 5], [2, 6], [3, 7],
            [8, 9], [8, 10], [8, 11]
        ]

        self.m_resLine = o3d.utility.Vector2iVector(listLine)
        self.m_mtrlNormal = rendering.MaterialRecord()
        self.m_mtrlNormal.shader = "defaultUnlit"  # to use line_width property
        self.m_mtrlNormal.base_color = (0, 0, 1, 1)
        self.m_mtrlNormal.point_size = 1
        self.m_mtrlCollision = rendering.MaterialRecord()
        self.m_mtrlCollision.shader = "defaultUnlit"  # to use line_width property
        self.m_mtrlCollision.base_color = (1, 0, 0, 1)
        self.m_mtrlCollision.point_size = 1

        self.m_key = key
        self.m_obb = scoMath.CScoOBB()
        self.m_geometry = o3d.geometry.LineSet()
        self.update_resource()

        '''
        0 : normal
        1 : collision
        '''
        self.m_state = 0
    
    def make_with_2_point(self, p0 : scoMath.CScoVec3, p1 : scoMath.CScoVec3, halfSize : scoMath.CScoVec3) :
        self.m_obb.make_with_2_point(p0, p1, halfSize)
        self.update_resource()

        ''' mesh test
        npVertex = np.array(
            [
                [-100, -100, 0],
                [100, -100, 0],
                [0, 100, 0]
            ]
        )
        npIndex = np.array(
            [
                [0, 1, 2]
            ]
        ).astype(np.int32)
        self.m_mesh = open3d.geometry.TriangleMesh()
        self.m_mesh.vertices = open3d.utility.Vector3dVector(npVertex)
        self.m_mesh.triangles = open3d.utility.Vector3iVector(npIndex)
        self.m_widget.scene.add_geometry("meshTest", self.m_mesh, self.m_mtrl)
        '''

    # protected
    def update_resource(self) :
        listVertex = [
            [-self.m_obb.HalfSize.X, self.m_obb.HalfSize.Y, 0],     # p0
            [-self.m_obb.HalfSize.X, -self.m_obb.HalfSize.Y, 0],    # p1
            [self.m_obb.HalfSize.X, -self.m_obb.HalfSize.Y, 0],     # p2
            [self.m_obb.HalfSize.X, self.m_obb.HalfSize.Y, 0],      # p3

            [-self.m_obb.HalfSize.X, self.m_obb.HalfSize.Y, self.m_obb.HalfSize.Z],     # p4
            [-self.m_obb.HalfSize.X, -self.m_obb.HalfSize.Y, self.m_obb.HalfSize.Z],    # p5
            [self.m_obb.HalfSize.X, -self.m_obb.HalfSize.Y, self.m_obb.HalfSize.Z],     # p6
            [self.m_obb.HalfSize.X, self.m_obb.HalfSize.Y, self.m_obb.HalfSize.Z],      # p7

            [0, 0, 0],                      # p8 origin
            [self.m_obb.HalfSize.X, 0, 0],  # p9
            [0, self.m_obb.HalfSize.Y, 0],  # p10
            [0, 0, self.m_obb.HalfSize.Z]   # p10
        ]

        self.m_geometry.points = o3d.utility.Vector3dVector(listVertex)
        self.m_geometry.lines = self.m_resLine

    @property
    def Geometry(self) :
        return self.m_geometry
    @property
    def Mtrl(self) :
        if self.State == 0 :
            return self.m_mtrlNormal
        else  :
            return self.m_mtrlCollision
    @property
    def WorldMatrix(self) :
        return self.m_obb.WorldMatrix
    @property
    def State(self) :
        return self.m_state
    @State.setter
    def State(self, state : int) :
        self.m_state = state
class CRenderObjCenterLine(CRenderObj) :
    def __init__(
            self, key : str, 
            sp0 : scoMath.CScoVec3, sp1 : scoMath.CScoVec3, sp2 : scoMath.CScoVec3,
            ep0 : scoMath.CScoVec3, ep1 : scoMath.CScoVec3, ep2 : scoMath.CScoVec3
                 ):
        super().__init__()
        # obb resource 
        listLine = [
            [0, 1], [1, 2]
        ]

        self.m_resLine = o3d.utility.Vector2iVector(listLine)
        self.m_mtrl = rendering.MaterialRecord()
        self.m_mtrl.shader = "defaultUnlit"  # to use line_width property
        self.m_mtrl.base_color = (1, 1, 0, 1)
        self.m_mtrl.point_size = 1

        self.m_key = key
        self.m_geometry = o3d.geometry.LineSet()
        self.m_worldMatrix = scoMath.CScoMat4()

        self.m_sp0 = sp0.clone()
        self.m_sp1 = sp1.clone()
        self.m_sp2 = sp2.clone()
        self.m_ep0 = ep0.clone()
        self.m_ep1 = ep1.clone()
        self.m_ep2 = ep2.clone()

        self.m_geometry.lines = self.m_resLine
        self.m_ray = scoMath.CScoRay()

    def process(self, ratio : float) :
        self.m_ray.make_with_point(self.m_sp0, self.m_ep0)
        p0 = self.m_ray.get_pos(ratio)
        self.m_ray.make_with_point(self.m_sp1, self.m_ep1)
        p1 = self.m_ray.get_pos(ratio)
        self.m_ray.make_with_point(self.m_sp2, self.m_ep2)
        p2 = self.m_ray.get_pos(ratio)

        listVertex = [
            [p0.X, p0.Y, p0.Z],     # p0
            [p1.X, p1.Y, p1.Z],     # p1
            [p2.X, p2.Y, p2.Z]      # p2
        ]

        self.m_geometry.points = o3d.utility.Vector3dVector(listVertex)

        return (p0, p1, p2)


    @property
    def Geometry(self) :
        return self.m_geometry
    @property
    def Mtrl(self) :
        return self.m_mtrl
    @property
    def WorldMatrix(self) :
        return self.m_worldMatrix


class CAPPRegistration :
    s_cameraDist = 300
    s_eapPath = "eap"
    #s_eapPath = "eapReg"
    s_ppPath = "pp"

    def __init__(self, title : str, width : int, height : int) -> None:
        self.m_bExit = False
        self.m_title = title
        self.m_width = width
        self.m_height = height

        self.m_app = gui.Application.instance
        self.m_app.initialize()
        self.m_win = self.m_app.create_window(title, width, height)

        self.init_mtrl()
        self.init_scene()
        self.init_render_obj()

        self.m_mainLayout = gui.Vert()
        '''
        self.m_mainLayout.add_child(self.init_selection_layout())
        self.m_mainLayout.add_fixed(30)
        self.m_mainLayout.add_child(gui.Label("Offset Info"))
        self.m_mainLayout.add_child(self.init_offset_layout())

        self.m_mainLayout.add_fixed(30)
        self.m_mainLayout.add_child(gui.Label("Dice Score Info"))
        self.m_mainLayout.add_child(self.init_dice_layout())
        '''

        self.m_win.add_child(self.m_widget)
        self.m_win.add_child(self.m_mainLayout)
        self.m_win.set_on_layout(self._on_layout)

        self.m_listRender = []

        self.state_init()
    def init_mtrl(self) :
        self.m_mtrl = rendering.MaterialRecord()
        self.m_mtrl.shader = "defaultUnlit"
        self.m_mtrl.base_color = (1, 0, 0, 1)
        self.m_mtrl.point_size = 10
    def init_scene(self) :
        self.m_widget = gui.SceneWidget()
        # scene 구성
        self.m_widget.scene = rendering.Open3DScene(self.m_win.renderer)
        self.m_widget.set_on_key(self._on_key)
        #self.m_widget.set_on_mouse(self._on_mouse)
        #self.m_widget.set_on_tick_event(self._on_tick_event)
    def init_render_obj(self) :
        self.m_obb = CRenderObjOBB("keyOBB")

        sp0 = scoMath.CScoVec3(0.0, 0.0, 0.0)
        ep0 = scoMath.CScoVec3(0.0, 0.0, 0.0)
        sp1 = scoMath.CScoVec3(1.0, -1.0, 1.0)
        ep1 = scoMath.CScoVec3(1.0, 1.0, 1.0)
        sp2 = scoMath.CScoVec3(2.0, -2.5, 2.0)
        ep2 = scoMath.CScoVec3(2.0, 2.5, 2.0)
        self.m_centerLine = CRenderObjCenterLine("keyCL", sp0, sp1, sp2, ep0, ep1, ep2)
        self.m_centerLine.process(0.0)

        listPt = [(3, 3, 3)]
        self.m_vecPt = scoMath.CScoVec3(listPt[0][2], listPt[0][1], listPt[0][0])
        self.m_pcdPoint = scoUtil.CScoUtilSimpleITK.get_pcd_from_list(listPt, (1, 1, 1))
    
    def clear(self) :
        self.m_widget.scene.clear_geometry()
        self.m_listRender.clear()

        # 추후에 3d_label도 remove 시켜야 함. 


    def process(self) :

        import threading
        import time

        #'''
        def update_geometry():
            self.m_widget.scene.clear_geometry()

            key = self.m_centerLine.Key
            geometry = self.m_centerLine.Geometry
            mtrl = self.m_centerLine.Mtrl
            self.m_widget.scene.add_geometry(key, geometry, mtrl)
            
            key = self.m_obb.Key
            geometry = self.m_obb.Geometry
            mtrl = self.m_obb.Mtrl 
            mat = self.m_obb.WorldMatrix
            self.m_widget.scene.add_geometry(key, geometry, mtrl)
            self.m_widget.scene.set_geometry_transform(key, mat.m_npMat)

            key = "keyPoint"
            self.m_widget.scene.add_geometry(key, self.m_pcdPoint, self.m_mtrl)
        def thread_main():
            fRatio = 0.0
            state = 0
            while self.m_bExit == False:
                p0, p1, p2 = self.m_centerLine.process(fRatio)
                halfSize = scoMath.CScoVec3(0.2, 0.2, 4)
                self.m_obb.make_with_2_point(p1, p2, halfSize)

                bRet = scoMath.CScoMath.intersect_obb_vec3(self.m_obb.m_obb, self.m_vecPt)
                if bRet == True :
                    self.m_obb.State = 1
                else :
                    self.m_obb.State = 0

                gui.Application.instance.post_to_main_thread(self.m_win, update_geometry)


                time.sleep(0.1)

                if state == 0 :
                    fRatio += 0.1
                    if fRatio > 1.0 :
                        fRatio = 1.0
                        state = 1
                else :
                    fRatio -= 0.1
                    if fRatio < 0.0 :
                        fRatio = 0.0
                        state = 0
        
        threading.Thread(target=thread_main).start()
        self.m_app.run()
        self.m_bExit = True
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
        pass

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
    def _on_mouse(self, e):
        if e.type == gui.MouseEvent.Type.BUTTON_DOWN:
            print("[debug] mouse:", (e.x, e.y))
        return gui.Widget.EventCallbackResult.IGNORED
    

    def state_init(self) :
        self.m_listRender.append(self.m_obb)
        self.m_listRender.append(self.m_centerLine)

        p0 = scoMath.CScoVec3(1, 0, 0)
        p1 = scoMath.CScoVec3(2, 1, 0)
        halfSize = scoMath.CScoVec3(1, 1, 3)
        self.m_obb.make_with_2_point(p0, p1, halfSize)
        #self.clear()
        #'''
        for renderObj in self.m_listRender :
            key = renderObj.Key
            geometry = renderObj.Geometry
            mtrl = renderObj.Mtrl 
            self.m_widget.scene.add_geometry(key, geometry, mtrl)
        for renderObj in self.m_listRender :
            key = renderObj.Key
            mat = renderObj.WorldMatrix
            self.m_widget.scene.set_geometry_transform(key, mat.m_npMat)
        #'''
        self.update_scene()
    def state_exe(self) :
        pass
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
    app = CAPPRegistration("OBB Test", 1280, 720)
    app.process()


print("clear ~~")



