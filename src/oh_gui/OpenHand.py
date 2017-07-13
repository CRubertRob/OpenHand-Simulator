#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# at your option, any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# This software is developed and maintained by Carlos Rubert (cescuder@uji.es) 
# at the Robotic Intelligence laboratory from Universitat Jaume I of Castellón (Spain)

from __future__ import with_statement # for python 2.5
from sympy.physics.units import ohm
from sympy.logic.boolalg import false
__copyright__ = 'OpenHand 2016'

import sys, os, re, logging, signal
from multiprocessing import Process,Pipe
from threading import Thread
from openravepy import *
#from mlabwrap import mlab
from numpy import *
from math import *
import oh_utils.oh_utils
import oh_utils.CalculateMeasures
import time
import scipy
import copy
import random

import h5py

import os
import shutil

from Ui_OpenHand import *
#from Ui_OpenHand_Graph import *
from PyQt4 import QtGui, QtCore, Qt
import ImageQt, popplerqt4

logger = None

#Define Paths
path_hand=os.path.dirname(os.path.abspath(__file__))+'/../../'+'env/hands/'     
path_obj=os.path.dirname(os.path.abspath(__file__))+"/../../env/objects/"
path_experiments=os.path.dirname(os.path.abspath(__file__))+"/../../env/experiments/"
print path_experiments
experiment="Default"

#Define Global Types
dt_array = h5py.special_dtype(vlen=numpy.dtype('float'))



class CallbackHandler(QtCore.QThread):
    def __init__(self,pipe,callback=None):
        super(CallbackHandler,self).__init__()
        self.callback = callback
        self.pipe = pipe

    def run(self):
        while(True):
            #print 'CallbackHandler:run'
            resultValue = self.pipe.recv()
            
            msg = [self.callback,resultValue]
            aux = self.emit(QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),msg)
        
class OpenRaveServer(object):
    '''
    classdocs
    '''

    def __init__(self,pipe):
        '''
        Constructor
        '''
        self.pipe = pipe
        self.running = True
        self.orenv = Environment()
        self._run()
        


    def __del__(self):
        RaveDestroy()

    def _run(self):
        while(self.running):
            (functionName,args,handleCallback) = self.pipe.recv()
            rValue,rMessage = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipe.send([rValue,rMessage])

    def executeFunction(self,name,args):
        rValue = None
        rMessage = "Function with "+name+" not available"
        if name in dir(self):
            if(args is None):
                rValue,rMessage = getattr(self,name)()
            else:
                rValue,rMessage = getattr(self,name)(args)
        return rValue,rMessage

    def StartGui(self):
        try:
            self.orenv.SetViewer('qtcoin')
            RaveSetDebugLevel(DebugLevel.Error)
            server = RaveCreateModule(self.orenv,'textserver')
            port = 4765
            while(self.orenv.AddModule(server,str(port))==-1):
                port+=1
            print "using port: "+str(port)

            self.HandProblem = None
            return True,None
        except Exception as e:
            print e
            pass
        return None,"Please start OpenRAVE first."

    def LoadEnv(self,env):
        try:
            self.orenv.Reset()
            if((env is not None) and self.orenv.Load(env) == False):
                return None, "Could not load "+env+"."
            return True,None
        except Exception as e:
            print e
            pass
        return None, "Please start OpenRAVE first."
    
    def LoadHand(self, hand):
        try:
            self.orenv.Reset()
            if (hand is not None):
                self.robot=self.orenv.ReadRobotURI(hand)                                
                self.orenv.Add(self.robot, True)             
                self.hand = self.orenv.GetRobots()[0]                  
            return True,None
        except Exception as e:
            print e
        return None, "Please start OpenRAVE first."
    
    def setHumanHandRobot(self):
        for i,robot in enumerate(self.orenv.GetRobots()):
            if robot.GetName() == 'HumanHand':                
                self.hand = self.orenv.GetRobots()[0]
            
    def LoadHandProblem(self):
        try:
            self.orenv.Reset()
            self.HandProblem = RaveCreateProblemInstance(self.orenv,'HandProblem')
            print self.HandProblem
            self.orenv.LoadProblem(self.HandProblem,'') 
            
            return True,None
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    
    def LoadArm(self, params):
        try:
            vparams = params.split(' ')
            HB=float(vparams[0])/1000;
            HL=float(vparams[1])/1000;
            
            cmd = 'sethandparameters '+ str(HB) + ' ' + str(HL)
            response = self.HandProblem.SendCommand(cmd);
            self.orenv.UpdatePublishedBodies()        
            self.setHumanHandRobot()   
            
            return True,None
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    def SetTransparency(self,params):
        try:
            with self.orenv:
                self.setHumanHandRobot()  
                for link in self.hand.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency(float(params))    
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
       
    def SetCamera(self,params):
        try:
            focal = 0.283434 # bigger values make orthographic view
                
            body = self.orenv.GetKinBody('object')        
                        
            self.setHumanHandRobot()     
            if self.hand is not None:        
                wrist = self.hand.GetLink("wrist_wrist")
                wristT_global = wrist.GetTransform()
                if body is not None: 
                    d = (wristT_global[1,3]-body.GetTransform()[1,3])*3
                    camerapos=wrist.GetTransform()[0:3,3]          
                    camerapos[1] = camerapos[1]-d
            else:
                camerapos = ones(3)*1.0/sqrt(3)
            
            if body is not None:
                Tcamera=transformLookat(lookat=body.GetTransform()[0:3,3],camerapos=camerapos,cameraup=[0,0,-1])
            else:
                Tall=transformLookat(lookat=zeros(3),camerapos=ones(3)*1.0/sqrt(3),cameraup=[0,0,-1])
                t = [0.944348, -0.095889, 0.494516]
                axis =array((0.749037, 0.036339, -0.661531))
                angle = 172.526638
                camfocal = 0.898456
                Tcamera = eye(4)
                Tcamera[0:3,3] = t
                Tcamera[0:3,0:3] = rotationMatrixFromQuat(quatFromAxisAngle(axis,angle*numpy.pi/180))           

            self.orenv.GetViewer().SetCamera(Tcamera,focal)             
            
        except Exception as e:
                print e
                pass 
        return None, "Please start OpenRAVE first."
    def SaveImage(self, params):
        try:
            imagedir =''
            imagename = str(params)
            width=1920
            height=1920
            focal = 1800 # bigger values make orthographic view

            K=[focal,focal,width/2,height/2]
            self.orenv.GetViewer().SendCommand('SetFiguresInCamera 1') 
            T = self.orenv.GetViewer().GetCameraTransform()
            I = self.orenv.GetViewer().GetCameraImage(width=width,height=height,transform=T,K=K)
      
            scipy.misc.pilutil.imsave(os.path.join(imagedir,imagename),hstack([I]))
        except Exception as e:
                print e
                pass 
        return None, "Please start OpenRAVE first."
    
    def SetPosture(self,params):
        try:
            self.setHumanHandRobot()  
            vparams = params.split(' ')
            value = vparams[0]
            joint = vparams[1]
            valueRad = float(value)*numpy.pi/180.0
            self.hand.SetDOFValues([float(valueRad)],[int(joint)])
            return True,None
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    def LoadObject(self,meshName):
        print "Loading Object"
        return oh_utils.oh_utils.LoadObject(self.orenv, meshName, path_obj), None
        
    
    def getContactPointNormal(self, depth, points, normals):
        
        handles = []
        contactsOUT=[]        
        friccion = 0.4 #Only to draw the cones
        maxDepth = 0.0
        centroide = [0, 0, 0]
        normal_centroide = [0, 0, 0]
        normal_centroide_old = [0, 0, 0]
                
        num_contacts = size(depth)
        for n in range(num_contacts):
            point=points[n*3:n*3+3]
            normal=normals[n*3:n*3+3]
            normal = normal*-1.0            
            centroide = centroide + point     
            normal_centroide =normal_centroide + normal      
        
        centroide= centroide/num_contacts
        normal_centroide = normal_centroide/num_contacts        
        
        return numpy.append(centroide, normal_centroide)

    def HumanGrasp(self, param=None):
        result=oh_utils.oh_utils.HumanGrasp(self.orenv)
        if result==None:
            return False, None
        contacts=result[0]
        print result[2]
        print result[3]
        for contact in contacts:
            print contact
        return True, result    
            
    def Grasp(self, param=None):  
        result=oh_utils.oh_utils.Grasp(self.orenv)
        if result==None:
            return False, None
        contacts=result[0]
        print "Contact Points Information"
        print "Fingers ID: " + str(result[2])
        print "Links ID: " + str(result[3])
        for contact in contacts:
            print "Contact Point i: " + str(contact)
        return True, result    
    
    def Open(self, param=None):  
        robot = self.orenv.GetRobots()[0]
        indices=self.hand.GetManipulators()[0].GetGripperIndices()
        robot.SetDOFValues(robot.GetDOFLimits()[0][indices], indices)
        
        return True, None
        
    def testGrasp(self, param=None):
        try:     
            target = self.orenv.GetKinBody('object') 
            robot = self.orenv.GetRobots()[0]
            self.orenv.GetCollisionChecker().SetCollisionOptions(CollisionOptions.Contacts)
            report=CollisionReport() 
            aux=self.orenv.CheckCollision(robot,target)            
            if aux==True:
                print "Pre-Grasp posture in collision"                
            else:           
                robot.WaitForController(0)
                taskmanip = interfaces.TaskManipulation(robot)
                robot.WaitForController(0)
                grasper_aux=interfaces.Grasper(robot)                 
                agarre=grasper_aux.Grasp(transformrobot=False,execute=False,outputfinal=True, target=target, forceclosure=True)               
                return self.Grasp(param)
            
        except Exception as e:
            print "Error Grasping:" + e            
            return True, 0.0        

        
    def QM(self, params):
        contactsVector=params[0];
        dits=params[1];
        links=params[2];
        object=self.orenv.GetKinBody('object') 
        robot = self.orenv.GetRobots()[0]
        measures=[]
        while len(measures)<1:
            measures=hstack(oh_utils.CalculateMeasures.CalculateMeasures(object, contactsVector, robot, dits, links))
        return True, measures
        
    def LoadPosture(self, params):
        posture=[]
        posture.append(params[0])
        posture.append(params[1:])
        return oh_utils.oh_utils.LoadPosture(self.orenv, posture), None
        
        
    def ShowBones(self):
        try:
            self.setHumanHandRobot()  
            xmlString = """
            <KinBody name="Bones">
                <Body name="bones" type="static">
                    <Geom type="trimesh">
                           <Render>bones.iv 0.1</Render>
                           <data>bones.iv 0.1</data>
                      </Geom>
                   </Body>
            </KinBody>
            """    
            with self.orenv:
                body = self.orenv.ReadKinBodyData(xmlString)
                body.SetName('bones')
                self.orenv.Add(body)
                
                wrist = self.hand.GetLink("wrist_wrist")
                wristT_global = wrist.GetTransform()
                body.SetTransform(wristT_global)
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    
    def GetPosture(self):
        try:
            self.setHumanHandRobot() 
            cmd = 'getposture'
            
            modules = self.orenv.GetModules()
            for mod in modules:
                if mod.GetXMLId() == 'HandProblem':
                    self.HandProblem = mod
            response = self.HandProblem.SendCommand(cmd);
            postureStr=""
            postures = response.split(';')
            res=postures[0].split(' ')
            postureStr += 'posture(1,:)=[\t'
            for i in range(len(res)-1):
                value = "%.4f" % float(res[i])
                
                postureStr += str(value)
                
                if (i+1)%6 == 0:
                    fila = ((i+1)/6)+1
                    postureStr += '\t];\n'
                    if fila < 6:
                        postureStr += 'posture(' + str(fila)+ ',:)=[\t'
                else:
                    postureStr += '\t'
            
            #ARM
            res=postures[1].split(' ')
            values=[]
            for i in range(len(res)-1):
                values.append("%.4f" % float(res[i]))
                
            postureStr += '\narm_posture(1,:) = [\t'+values[0]+'\t'+values[1]+'\t'+values[2]+'\t];'
            postureStr += '\narm_posture(2,:) = [\t'+values[3]+'\t'+values[4]+'\t'+values[5]+'\t];'
            postureStr += '\narm_posture(3,:) = [\t'+values[6]+'\t'+values[7]+'\t'+values[8]+'\t];'
            
            return True,postureStr
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    
    def GetRobotPosture(self):
        try:
            self.setHumanHandRobot()
            self.hand=self.orenv.GetRobots()[0]
            cmd = 'getposture'

            postureStr=""  
            values=self.hand.GetDOFValues()
            postureStr += '\nposture_0 = ['
            for i in values:
                postureStr += '\t'+'%.4f'%i
            postureStr += '];\n'
            
            values=self.hand.GetTransform()
            postureStr += '\nhandT = ['
            for i in values:
                for j in i:
                    postureStr += '\t'+'%.4f'%j
                postureStr += '\n'
            postureStr += '];\n'
            
            return True,postureStr
        except Exception as e:
            print e
            pass 
        return None, "Please start OpenRAVE first."
    
    def consumidor(self, gmodel, (approachray, roll, preshape, standoff, manipulatordirection)):
        grasp = zeros(gmodel.totaldof)
        grasp[gmodel.graspindices.get('igrasppos')] = approachray[0:3]
        grasp[gmodel.graspindices.get('igraspdir')] = -approachray[3:6]
        grasp[gmodel.graspindices.get('igrasproll')] = roll
        grasp[gmodel.graspindices.get('igraspstandoff')] = standoff
        grasp[gmodel.graspindices.get('igrasppreshape')] = preshape
        grasp[gmodel.graspindices.get('imanipulatordirection')] = manipulatordirection
        try:
            ini=time.time()
            contacts,finalconfig,mindist,volume = gmodel.runGrasp(grasp=grasp, forceclosure=True)
            
        except planning_error, e:
            print 'Grasp Failed: '
            return None
        if mindist>=1e-9:
            return finalconfig[1]
        else:
            return None
    
    def GenGrasps(self, params):
        experiment=params[7]
        if not os.path.exists(path_experiments+experiment+'/'):
            os.makedirs(path_experiments+experiment+'/') 
        
        hands=params[0]
        objects=params[1]
        standoffs=params[2]
        rolls=params[3]
        boxdelta=params[4]
        normalanglerange=params[5]
        maxgrasps=params[6]        
        for hand in hands:
            print hand
            if hand=="HumanHand":
                self.LoadHandProblem()                
                HB="78.0"
                HL="172.0"                
                self.LoadArm(HB +' '+ HL)       
            else:                
                robot=self.LoadHand(path_hand+hand)
            for element in objects:
                object=element.split('.')[0]
                self.LoadObject(object)                             
                robot = self.orenv.GetRobots()[0] # get the first robot            
                target = self.orenv.GetKinBody('object') 
                print hand
                print object    
                gmodel = databases.grasping.GraspingModel(robot,target)
                gmodel.numgrasps=maxgrasps                          
                preshapes=[robot.GetDOFValues()]
                print robot.GetDOFValues()
                if hand=="barretthand.robot.xml":
                    for i in range(2,4):
                        preshape=robot.GetDOFValues()
                        preshape[3]=pi/i
                        preshapes.append(preshape)                      
                elif hand=="barretthand4fingers.robot.xml":
                    for i in range(2,5):
                        preshape=robot.GetDOFValues()
                        preshape[4]=pi/2/i
                        preshape[5]=pi/2/i
                        preshapes.append(preshape)
                elif hand=="schunk.robot.xml":
                    for i in range(2,4):
                        preshape=robot.GetDOFValues()
                        preshape[0]=pi/2/i
                        preshapes.append(preshape)
                 
                gmodel.generate(boxdelta=boxdelta, normalanglerange=normalanglerange, standoffs=standoffs, rolls=rolls)
                 
                muestra=gmodel.grasps
                        
                #Store postures and measures for a GraspSet
                hand_file=h5py.File(path_experiments+'/'+experiment+'/'+hand.split('.')[0]+'.hdf5', 'a')
                obj_grp=hand_file.require_group(object.split('.')[0])
                grasps=obj_grp.require_dataset("grasps", (len(muestra),5), chunks=True, dtype=dt_array, compression="gzip", compression_opts=9)
                
                i=0                
                for item in muestra:                                
                    Tmanip=gmodel.manip.GetTransform()
                    preshape=item[gmodel.graspindices.get('igrasppreshape')]
                    if len(preshape)!= len(robot.GetDOFValues()):
                        preshape=preshapes[0]
                    Tmanip[0:3,3] += dot(Tmanip[0:3, 0:3], item[gmodel.graspindices.get('igrasptranslationoffset')])
                    transform=(dot(gmodel.getGlobalGraspTransform(item),dot(linalg.inv(Tmanip),gmodel.robot.GetTransform())))
                    grasps[i,0]=preshape                    
                    grasps[i,1:]=transform   
                    i+=1                
                self.orenv.Remove(self.orenv.GetKinBody('object') )
                fin=time.time()
                hand_file.close()
            study_file=h5py.File(path_experiments+'/'+experiment+'/'+experiment+'.hdf5', 'a')
            if hand.split('.')[0] not in study_file.keys():
                study_file[hand.split('.')[0]]=h5py.ExternalLink(path_experiments+experiment+'/'+hand+'.hdf5', '/')  
            study_file.close()
                        
        return True, None
    
    def getPosture(self):
        params=[]
        robot = self.hand
        params.append(robot.GetDOFValues())
        params.append(robot.GetTransform())
        return True, params
    
    def quit(self):
        RaveDestroy()
        self.running = False
        return True,None

class Server(object):
    '''
    Control server to run the benchmark in its own process.
    '''

    def __init__(self):
        '''
        Setup the shared memory data structure model and initialize the control parts.
        '''        
        self.openRaveServers = []
        self.running = True
        self.orgui = None
        self.qtgui = None
        self.object = None
        (self.pipeQtControl, self.pipeORControl) = Pipe()
        (self.pipeQtServer, self.pipeServer) = Pipe()
        self.StartQtGuiControl()
        self._run()
        
    def _run(self):
        '''
        Main server loop which waits for input from the qt-gui.
        '''
        while(self.running):
            (functionName,args,handleCallback) = self.pipeServer.recv()      
            rValue = self.executeFunction(functionName, args)
            if handleCallback:
                self.pipeServer.send([rValue, rValue])

    def executeFunction(self,name,args):
        if name == "___START_OPENRAVE_SERVER___":
            return self.StartOpenRaveGuiServer()
        if name == "___READ_POSTURE___":
            try:
                vparams = args.split(' ')
                file=vparams[0];
                path=vparams[1];
                HB=vparams[2];
                HL=vparams[3];
                qm = '0'
                print "loading posture " + str(file)
                output = mlab.TestPostureGui(path, '', '', file,'0',qm, HB, HL)        
                
            except Exception as e:
                print e
                pass    
            return True
        if name == "___GRASP___":
            try:                
                print('Grasp in progress...')      
                output = mlab.TestPostureGui()
                if qm=='1':
                    print ("Processing measures")
                    measures=output.measures              
                    measures=numpy.vstack((measures, 1))
                    print "Quality Results"
                    print measures                    
                    X=[measures[3], measures[8], measures[9], measures[10], measures[2], measures[6]]                
                    mlab.RadarGraph(X, "Graph")
                    output = mlab.ClassifyMeasure(measures)
                    measures=numpy.vstack((measures, output))
                    print('done')                
                    return measures
                else:
                    print('done')                
                    return None
            
            except Exception as e:
                print e
                pass    
            return True     
        if name == "___QM___":
            try:
                contactsVector=args[0];                
                dits=args[1];
                links=args[2];         
                output = mlab.TestExperiments_TFM(contactsVector, dits, links)                
                measures=output.measures   
                X=[measures[3], measures[8], measures[9], measures[2], measures[6]]
                mlab.RadarGraph(X, "Graph")
                return measures
            except Exception as e:
                print e
                pass    
            return True              
        if name == "___CLEAR_MARKERS___":
            try:
                mlab.orEnvClose()
            except Exception as e:
                print e
                pass    
            return True
        if name == "___CLOSE___":
            self.running = False
            try:
                self.qtgui.terminate()
                self.orgui.terminate()
            except:
                pass
            return True
        return False   



    def norm(self, value, max, min):
        return (value-min)/(max-min)
    
    def normalize(self, measures):
        minU=[0.0, 0.0, 0.0, 0.07, 0.0, 0.27, 0.0, 0.0, 0.51, 0.0, 0.81, 0.08]
        maxU=[0.52, 11.30, 0.24, 1.0, 0.36, 1.0, 0.09, 0.03, 0.94, 0.02, 1.0, 1.0]
        res=[]        
        for i in xrange(len(measures)):  
            measures[i]=self.norm(measures[i][0], maxU[i%10], minU[i%10])
        return measures  
    
    def normalizeRobot(self, measures):
        minU=[0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.43, 0.0]
        maxU=[0.98, 11.30, 0.57, 1.0, 1.63, 1.0, 0.1, 0.06, 1, 0.08]
        res=[]        
        for i in xrange(len(measures)):  
            measures[i]=self.norm(measures[i][0], maxU[i%10], minU[i%10])
        return measures        
        
    def StartOpenRaveGuiServer(self):
        if self.orgui:
            self.orgui.terminate()
        self.orgui = Process(target=OpenRaveServer,args=(self.pipeORControl,))
        self.orgui.start()
        return True

    def StartQtGuiControl(self):
        self.qtgui = Process(target=self._StartQtGuiControl)
        self.qtgui.start()
        return True

    def _StartQtGuiControl(self):
        app = QtGui.QApplication(sys.argv)
        form = MainWindow(self.pipeQtControl,self.pipeQtServer)
        form.show()
        app.exec_()

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s


class MainWindow(QtGui.QMainWindow, Ui_OpenHand):

    @QtCore.pyqtSignature("")
    def on_pbClose_clicked(self):
        self.closeAll()

    def __init__(self,pipeOR,pipeServer):
        super(MainWindow,self).__init__(None)
        self.pipeServer = pipeServer
        self.pipeOR = pipeOR

        
        
        self.CallbackHandler = CallbackHandler(self.pipeOR)
        self.connect(self.CallbackHandler,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.CallbackHandlerServer = CallbackHandler(self.pipeServer)
        self.connect(self.CallbackHandlerServer,QtCore.SIGNAL("CallbackHandler(PyQt_PyObject)"),self.HandleCallback)
        self.__index = 0
        self.setupUi(self)
        
        #Disable Buttons
        self.pbLoadMarkers.setEnabled(False)
        self.pbLoadInitial.setEnabled(False)
        self.pbLoadEnd.setEnabled(False)
        self.pbGrasp.setEnabled(False)
        self.pbQM.setEnabled(False)
        
        #Disable Buttons Robot
        self.pbLoadHand.setEnabled(False)
        self.pbRLoadObject.setEnabled(False)
        self.pbRLoadMarkers.setEnabled(False)
        self.pbRInitialPosture.setEnabled(False)
        self.pbRGrasp.setEnabled(False)
        self.pbRAll.setEnabled(False)
        
        #load comboboxes
        self.directory = QtCore.QDir("~/")
        self.umano_path = os.path.dirname(os.path.abspath(__file__))+'/../../'     
        
        self.directory.setPath(self.umano_path+'env/objects/')      
        self.cBLoadObject.clear()
        self.cBLoadObject.addItems(self.directory.entryList())
        
        #self.directory.setPath(self.umano_path+'env/objects/');
        self.directory.setPath(path_obj)      
        self.cBObject.clear()
        self.cBObject.addItems(self.directory.entryList())
        
        self.directory.setPath(self.umano_path+'env/scenes/');      
        self.cBLoadEnv.clear()
        self.cBLoadEnv.addItems(self.directory.entryList())
        
        #HUMANO        
        self.directory.setPath(self.umano_path+'env/postures/markers')     
        self.cBLoadMarkers.clear()
        self.cBLoadMarkers.addItems(self.directory.entryList())
        
        self.directory.setPath(self.umano_path+'env/postures/markers')     
        self.cBMarkers.clear()
        self.cBMarkers.addItems(self.directory.entryList())      
        
        self.LoadComboBoxes()         
        
        self.directory.setPath(path_experiments);      
        self.cBRLoadExp.clear()
        self.cBRLoadExp.addItems(self.directory.entryList()[2:])
        
        #GRASP GENERATOR  
        self.directory.setPath(path_hand);      
        self.lWHands.clear()
        self.lWHands.addItem("All")
        self.lWHands.addItems(self.directory.entryList()[2:])
        self.lWHands.addItems(["HumanHand"])
        
        self.directory.setPath(path_obj);   
        self.lWObjects.clear()
        self.lWObjects.addItem("All")
        self.lWObjects.addItems(self.directory.entryList()[2:])
        
        self.directory.setPath(path_experiments);      
        self.cBGExperiment.clear()
        self.cBGExperiment.addItem("NEW")
        self.cBGExperiment.addItems(self.directory.entryList()[2:])
        
        self.directory.setPath(path_hand);      
        self.cBGHand.clear()
        self.cBGHand.addItems(self.directory.entryList()[2:])
        self.cBGHand.addItems(["HumanHand"])
        
        self.directory.setPath(path_obj);      
        self.cBGObject.clear()
        self.cBGObject.addItems(self.directory.entryList()[2:])
        
        
        #BENCHMARKING
        #Set mean values for all metrics
        self.style0=" QProgressBar::chunk {     background-color: white } QProgressBar {     border: 2px solid grey;     border-radius: 5px;     text-align: center;     background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,                 stop:0 red, stop: 0.5 yellow, stop:1 green); }"
        self.style1=" QProgressBar::chunk {     background-color: white } QProgressBar {     border: 2px solid grey;     border-radius: 5px;     text-align: center;     background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,                 stop:0 yellow, stop: 0.5 green, stop:1 cyan); }"
        self.style2=" QProgressBar::chunk {     background-color: white } QProgressBar {     border: 2px solid grey;     border-radius: 5px;     text-align: center;     background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,                 stop:0 green, stop: 0.5 cyan, stop:1 blue); }"
        self.QC=[0.2104, 0.5808, 0.0953, 0.4668, 0.0200, 0.4689, 0.0528]
        
        self.labels_1=[self.tLQC1_1,self.tLQC2_1,self.tLQC3_1,self.tLQC4_1,self.tLQC5_1,self.tLQC6_1,self.tLQC7_1]
        self.progress_1=[self.pBQC1_1,self.pBQC2_1,self.pBQC3_1,self.pBQC4_1,self.pBQC5_1,self.pBQC6_1,self.pBQC7_1]
        
        self.labels_2=[self.tLQC1_2,self.tLQC2_2,self.tLQC3_2,self.tLQC4_2,self.tLQC5_2,self.tLQC6_2,self.tLQC7_2]
        self.progress_2=[self.pBQC1_2,self.pBQC2_2,self.pBQC3_2,self.pBQC4_2,self.pBQC5_2,self.pBQC6_2,self.pBQC7_2]
        
        self.directory.setPath(path_experiments);      
        self.cBBenchExp.clear()        
        self.cBBenchExp.addItems(self.directory.entryList()[2:])
        
        self.pbBenchEval1.setEnabled(False)
        self.pbBenchEval2.setEnabled(False)
        
        #start OR automatically the first time
        self.updateTeOutput("Starting OpenRave.")
        self.SendToServer("___START_OPENRAVE_SERVER___")
        self.updateTeOutput("Starting qtcoin gui.")
        self.SendToOR("StartGui")      
        
        #Default values
        self.HB=78 
        self.HL=172
        self.graspFingers=[0,0,0,0,0]
        
        self.actual=0
        self.postures=[]
        self.ini=0
        self.fin=0
        self.j_sen=61
        self.transform=[]
        self.pos=""
        self.hdf5file=None
        
    def close(self):
        self.SendToOR("___CLOSE___")
        self.SendToServer("___CLOSE___")
        QtGui.QApplication.quit()

    def closeEvent(self, event):
        self.close()
        
    def LoadComboBoxes(self):
        self.directory.setPath(self.umano_path+'env/postures/Humano/Initial')     
        self.cBInitialPosture.clear()
        self.cBInitialPosture.addItems(self.directory.entryList())
        
        self.directory.setPath(self.umano_path+'env/postures/Humano/Initial')     
        self.cBLoadInitial.clear()
        self.cBLoadInitial.addItems(self.directory.entryList())
        
        self.directory.setPath(self.umano_path+'env/postures/Humano/End')     
        self.cBEndPosture.clear()
        self.cBEndPosture.addItems(self.directory.entryList())      
        
        self.directory.setPath(self.umano_path+'env/postures/Humano/End')     
        self.cBLoadEnd.clear()
        self.cBLoadEnd.addItems(self.directory.entryList())         

    #Load comboBox Experiments for Robot Hands        
    def LoadcBExperiments(self, envName):
        hand=envName.split('.')
        self.hand=hand[0]
        self.robot_path=self.umano_path+'env/postures/'+hand[0]
        
        self.directory.setPath(self.robot_path+'/Initial/')     
        self.cBRInitialPosture.clear()
        self.cBRInitialPosture.addItems(self.directory.entryList())

    #Load comboBox Postures for Robot Hands        
    def LoadcBPostures(self, envName):
        hand=envName.split('.')
        self.hand=hand[0]
        self.robot_path=self.umano_path+'env/postures/'+hand[0]
        
        self.directory.setPath(self.robot_path+'/Initial/')     
        self.cBRInitialPosture.clear()
        self.cBRInitialPosture.addItems(self.directory.entryList())        

    @QtCore.pyqtSignature("")
    def on_pbGClose_clicked(self):
        if self.handName=="HumanHand":
            self.SendToOR("HumanGrasp", None, None)
        else:
            self.SendToOR("Grasp", None, None)
        
    @QtCore.pyqtSignature("")
    def on_pbGOpen_clicked(self):
        self.SendToOR("Open", None, None)

    @QtCore.pyqtSignature("")
    def on_pbStartOpenRAVE_clicked(self):
        self.updateTeOutput("Starting OpenRave.")
        self.SendToServer("___START_OPENRAVE_SERVER___")
        self.updateTeOutput("Starting qtcoin gui.")
        self.SendToOR("StartGui")
        self.clearWindow()
    
    @QtCore.pyqtSignature("")
    def on_pbLoadEnd_clicked(self):   
        posture = str(self.cBLoadEnd.currentText()).split('.')
        postureFile = posture[0]      
        posturePath = str(self.umano_path+'env/postures/Humano/End')
        self.SendToServer("___READ_POSTURE___",str(postureFile)+' '+str(posturePath)+' '+str(self.HB)+' '+str(self.HL))
        
        self.pbGrasp.setEnabled(True)
        self.pbQM.setEnabled(True)
        
    @QtCore.pyqtSignature("")
    def on_pbLoadInitial_clicked(self):   
        posture = str(self.cBLoadInitial.currentText()).split('.')
        postureFile = posture[0]      
        posturePath = str(self.umano_path+'env/postures/Humano/Initial')
        self.SendToServer("___READ_POSTURE___",str(postureFile)+' '+str(posturePath)+' '+str(self.HB)+' '+str(self.HL))

        self.pbGrasp.setEnabled(True)
        self.pbQM.setEnabled(True)
        
    @QtCore.pyqtSignature("")
    def on_pbInitialPosture_clicked(self):   
        posture = str(self.cBEndPosture.currentText()).split('.')
        postureFile = posture[0]      
        posturePath = str(self.umano_path+'env/postures/Humano/Initial')
        print posturePath
        self.SendToServer("___READ_POSTURE___",str(postureFile)+' '+str(posturePath)+' '+str(self.HB)+' '+str(self.HL))

    @QtCore.pyqtSignature("")
    def on_pbEndPosture_clicked(self):   
        posture = str(self.cBEndPosture.currentText()).split('.')
        postureFile = posture[0]      
        posturePath = str(self.umano_path+'env/postures/Humano/End')
        self.SendToServer("___READ_POSTURE___",str(postureFile)+' '+str(posturePath)+' '+str(self.HB)+' '+str(self.HL))

    @QtCore.pyqtSignature("")
    def on_pbRInitialPosture_clicked(self):
        posture_num=int(str(self.cBRInitialPosture.currentText()).split()[1])
        posture=self.obj_grp["grasps"][posture_num]
        self.SendToOR("LoadPosture", posture, None)        
        self.pbRGrasp.setEnabled(True)

    @QtCore.pyqtSignature("")
    def on_pbGrasp_clicked(self): 
        postureI = str(self.cBLoadInitial.currentText()).split('.')
        postureE = str(self.cBLoadEnd.currentText()).split('.')  
        markers = str(self.cBLoadMarkers.currentText()).split('.')
        postureFileI = postureI[0]
        postureFileE = postureE[0]          
        postureFileM = markers[0]
        posturePath = str(self.umano_path+'env/postures/Humano/')
        self.SendToOR("HumanGrasp", None, None)      

    @QtCore.pyqtSignature("")
    def on_pbQM_clicked(self): 
        postureI = str(self.cBLoadInitial.currentText()).split('.')
        postureE = str(self.cBLoadEnd.currentText()).split('.')
        markers = str(self.cBLoadMarkers.currentText()).split('.')
        postureFileI = postureI[0]
        postureFileE = postureE[0]
        postureFileM = markers[0]
        posturePath = str(self.umano_path+'env/postures/Humano/')              
        self.SendToServer("___GRASP___",str(postureFileM)+' '+str(postureFileI)+' '+str(postureFileE)+' '+str(posturePath)+' '+str(self.HB)+' '+str(self.HL)+' '+'1', callback=self.updateMeasures)

    @QtCore.pyqtSignature("")
    def on_pbRGrasp_clicked(self): 
        self.ini=time.time()
        self.cleanValues()
        self.posture_num=self.cBRInitialPosture.currentIndex()
        if self.handName=="HumanHand":
            self.SendToOR("HumanGrasp", None, self.measure)
        else:
            self.SendToOR("Grasp", None, self.measure)
        
         
    @QtCore.pyqtSignature("")
    def on_pbRAll_clicked(self):        
        self.ini=time.time()
        self.pos=0
        self.values=[]
        posture=self.obj_grp["grasps"][self.pos]
        self.cleanValues()
        self.posture_num=0
        self.SendToOR("LoadPosture", posture, self.graspAll)
        
            
    @QtCore.pyqtSignature("")
    def on_pbSetTransparency_clicked(self): 
        trans = str(self.textTransparency.toPlainText())
        self.SendToOR("SetTransparency",trans)
    
    @QtCore.pyqtSignature("")
    def on_pbSetCameraView_clicked(self):
        self.SendToOR("SetCamera",str(0))

    @QtCore.pyqtSignature("")
    def on_pbShowBones_clicked(self):
        self.SendToOR("ShowBones")
    
    @QtCore.pyqtSignature("")
    def on_pbBrowsePath_clicked(self): 
        #load a directory 
        path = QtGui.QFileDialog.getExistingDirectory(self,"Directory", self.directory.path())
        print 'path after dialog: ',path 
        print self.directory.path()
        
    @QtCore.pyqtSignature("")
    def on_pbLoadObject_clicked(self): 
        meshName = str(self.cBLoadObject.currentText()).split('.')[0]        
        self.SendToOR("LoadObject",meshName)
        self.pbLoadMarkers.setEnabled(True)
        self.pbLoadInitial.setEnabled(True)
        self.pbLoadEnd.setEnabled(True)


    @QtCore.pyqtSignature("")
    def on_pbObject_clicked(self): 
        meshName = str(self.cBObject.currentText()).split('.')[0]       
        print meshName
        self.SendToOR("LoadObject",meshName) 
        
    @QtCore.pyqtSignature("")
    def on_pbRLoadObject_clicked(self): 
        meshName = str(self.cBRLoadObject.currentText())
            
        self.SendToOR("LoadObject",meshName)    
        
        self.obj_grp=self.hdf5file[meshName]
        self.meshName=meshName         
        self.cBRInitialPosture.clear()
        grasps_count=len(self.obj_grp['grasps'])
        posturesList=[]
        for i in range(grasps_count):
            posturesList.append("Posture %d"%i)            
        self.cBRInitialPosture.addItems(posturesList)
        self.pbRInitialPosture.setEnabled(True)
        self.pbRAll.setEnabled(True)
            
    @QtCore.pyqtSignature("")
    def on_pbRLoadExp_clicked(self): 
        expName = str(self.cBRLoadExp.currentText())
        path = path_experiments+expName+"/"
        
        self.experiment=expName
        if self.hdf5file!=None:self.hdf5file.close()
        self.hdf5file=h5py.File(path+expName+".hdf5", 'r')
        
        self.cBSelHand.clear()
        self.cBSelHand.addItems(self.hdf5file.keys())
        self.pbLoadHand.setEnabled(True)
        
    @QtCore.pyqtSignature("")
    def on_pbBenchExp_clicked(self): 
        expName = str(self.cBBenchExp.currentText())
        path = path_experiments+expName+"/"
        
        self.experiment=expName
        if self.hdf5file!=None:self.hdf5file.close()
        self.hdf5file=h5py.File(path+expName+".hdf5", 'r')
        
        self.cBBenchHand1.clear()
        self.cBBenchHand1.addItems(self.hdf5file.keys())
        self.pbBenchEval1.setEnabled(True)
        
        self.cBBenchHand2.clear()
        self.cBBenchHand2.addItems(self.hdf5file.keys())
        self.pbBenchEval2.setEnabled(True)
    
        
    @QtCore.pyqtSignature("")
    def on_pbLoadHand_clicked(self): 
        envName = str(self.cBSelHand.currentText())
        if envName=="HumanHand":
            self.on_pbLoadArm_clicked()
        else:
            if "shadow" in envName:
                path = path_hand+envName+".dae"
            else:
                path = path_hand+envName+".robot.xml"
            print "Load hand"
            
            self.SendToOR("LoadHand",path)
        
        self.hdf5file.close()
        self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+envName+".hdf5", "a")
        self.handName=envName
        self.cBRLoadObject.clear()
        self.cBRLoadObject.addItems(self.hdf5file.keys())
        self.pbRLoadObject.setEnabled(True)
        
    @QtCore.pyqtSignature("")
    def on_pbGHand_clicked(self): 
        handName = str(self.cBGHand.currentText())    
        if handName=="HumanHand":
            self.on_pbLoadArm_clicked()
        else:        
            self.SendToOR("LoadHand",path_hand+handName)    
        self.handName=handName.split('.')[0]
        
    @QtCore.pyqtSignature("")
    def on_pbGObject_clicked(self): 
        meshName = str(self.cBGObject.currentText()).split('.')[0]
        self.SendToOR("LoadObject",meshName)
        self.object= meshName   
        
    @QtCore.pyqtSignature("")
    def on_pbGExperiment_clicked(self): 
        expName = str(self.cBGExperiment.currentText())
        if expName!="NEW":            
            path = path_experiments+expName+"/"
            
            self.experiment=expName
            if self.hdf5file!=None:self.hdf5file.close()
            
            self.hdf5file=h5py.File(path+expName+".hdf5", 'a')
            if self.handName not in self.hdf5file.keys():                 
                self.hdf5file.create_group(self.handName)
            self.hdf5file.close()
            self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+self.handName+".hdf5", "a")
            
            if self.object not in self.hdf5file.keys():
                self.hdf5file.create_group(self.object)
            self.obj_grp=self.hdf5file[self.object]        
            self.cBGPosture.clear()
            if "grasps" in self.obj_grp.keys():
                grasps_count=len(self.obj_grp['grasps'])
            else:
                grasps_count=0
            posturesList=[]
            for i in range(grasps_count):
                posturesList.append("Posture %d"%i)            
            self.cBGPosture.addItems(posturesList)   
            
    @QtCore.pyqtSignature("")
    def on_pbGPosture_clicked(self):
        if self.cBGPosture.currentText()!=None:
            self.posture_num=int(str(self.cBGPosture.currentText()).split()[1])
            posture=self.obj_grp["grasps"][self.posture_num]            
            self.SendToOR("LoadPosture", posture, None)
    
    @QtCore.pyqtSignature("")
    def on_pbGSave_clicked(self):  
        self.SendToOR("getPosture", None, self.storePosture)
        
    @QtCore.pyqtSignature("")
    def on_pbGOverwrite_clicked(self):
        self.SendToOR("getPosture", None, self.overwritePosture)
        
    
    def createExperiment(self):
        text, ok = QtGui.QInputDialog.getText(self, 'New Experiment', 'Experiment Name:')
          
        if ok:
            self.experiment=str(text)
            os.makedirs(path_experiments+self.experiment)
            self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+self.experiment+".hdf5", 'a')                 
            self.hdf5file.create_group(self.handName)
            self.hdf5file.close()
            self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+self.handName+".hdf5", "a")
            self.hdf5file.create_group(self.object)
            self.obj_grp=self.hdf5file[self.object] 
            self.directory.setPath(path_experiments);      
            self.cBGExperiment.clear()
            self.cBGExperiment.addItem("NEW")
            self.cBGExperiment.addItems(self.directory.entryList()[2:])
            return True
        else: return False
    
    def storePosture(self, params):   
        if str(self.cBGExperiment.currentText())=="NEW":
            if self.createExperiment() == False:
                return False
            
        if "grasps" in self.obj_grp.keys():
            grasp_num=len(self.obj_grp["grasps"])
            grasps=self.obj_grp.require_dataset("grasps", (grasp_num,5),  chunks=True, dtype=dt_array, compression="gzip", compression_opts=9)
            grasps.resize((grasp_num+1, 5))
        else:
            grasp_num=0            
            grasps=self.obj_grp.create_dataset("grasps", (1,5),  maxshape=(None, 5), chunks=True, dtype=dt_array, compression="gzip", compression_opts=9)
        self.updateTeOutput("Overwriting Posture %d in Experiment: %s, Hand: %s, Object: %s"%(grasp_num, self.experiment, self.handName, self.obj_grp.name))
        grasps[grasp_num,0]=params[0]
        grasps[grasp_num,1:]=params[1]
        self.hdf5file.close()
        self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+self.handName+".hdf5", "a")
        self.obj_grp=self.hdf5file[self.object]  
        
    def overwritePosture(self, params):  
        self.updateTeOutput("Overwriting Posture %d in Experiment: %s, Hand: %s, Object: %s"%(self.posture_num, self.experiment, self.handName, self.obj_grp.name))
        grasp_num=self.posture_num
        grasps_count=len(self.obj_grp['grasps'])
        grasps=self.obj_grp.require_dataset("grasps", (grasps_count,5),  chunks=True, dtype=dt_array, compression="gzip", compression_opts=9)
        
        print params[1]
        grasps[grasp_num,0]=params[0]
        grasps[grasp_num,1:]=params[1]
        self.hdf5file.close()
        self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+self.handName+".hdf5", "a")
        self.obj_grp=self.hdf5file[self.object]  
        
    @QtCore.pyqtSignature("")
    def on_pbSaveImage_clicked(self):
        imageName = str(self.textSaveImage.toPlainText())
        self.SendToOR("SaveImage",imageName)
    


    @QtCore.pyqtSignature("")
    def on_pbClearMarkers_clicked(self): 
        self.SendToServer("___CLEAR_MARKERS___")
    
    @QtCore.pyqtSignature("")
    def on_pbGetPosture_clicked(self):
        self.SendToOR("getPosture",callback=self.updateTePosture)
    
    @QtCore.pyqtSignature("")
    def on_pbUpdateSliders_clicked(self):
        self.SendToOR("GetPosture", callback=self.updateSliders)
            
    @QtCore.pyqtSignature("")
    def on_pbLoadArm_clicked(self):
        self.updateTeOutput("Loading HandModule.")
        self.SendToOR("LoadHandProblem")
        self.updateTeOutput("Loading Arm.")
        self.HB="78.0"
        self.HL="172.0"
        self.handName="HumanHand"
        self.SendToOR("LoadArm", self.HB +' '+ self.HL)       
        
    #ARM_Joints
    @QtCore.pyqtSignature("int")
    def on_slider_fi_s_sliderMoved(self,value):
        joint = 0
        self.text_fi_s.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
    
    @QtCore.pyqtSignature("int")
    def on_slider_theta_s_sliderMoved(self,value):
        joint = 2
        self.text_theta_s.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
    
    @QtCore.pyqtSignature("int")
    def on_slider_psi_s_sliderMoved(self,value):
        joint = 1
        self.text_psi_s.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        

    @QtCore.pyqtSignature("int")
    def on_slider_psi_fe_e_sliderMoved(self,value):
        joint = 3
        self.text_psi_fe_e.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))        
        
    @QtCore.pyqtSignature("int")
    def on_slider_psi_ps_e_sliderMoved(self,value):
        joint = 4
        self.text_psi_ps_e.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    @QtCore.pyqtSignature("int")
    def on_slider_psi_fe_w_sliderMoved(self,value):
        joint = 5
        self.text_psi_fe_w.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
    
    @QtCore.pyqtSignature("int")
    def on_slider_psi_abd_w_sliderMoved(self,value):
        joint = 6
        self.text_psi_abd_w.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    #HAND Joints
    @QtCore.pyqtSignature("int")
    def on_slider_flexCMCthumb_sliderMoved(self,value):
        joint = 10
        self.text_flexCMCthumb.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdCMCthumb_sliderMoved(self,value):
        joint = 11
        self.text_abdCMCthumb.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))  
    
    @QtCore.pyqtSignature("int")
    def on_slider_flexMCPthumb_sliderMoved(self,value):
        joint = 8
        self.text_flexMCPthumb.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdMCPthumb_sliderMoved(self,value):
        joint = 9
        self.text_abdMCPthumb.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
            
    @QtCore.pyqtSignature("int")
    def on_slider_flexIPthumb_sliderMoved(self,value):
        joint = 7
        self.text_flexIPthumb.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
    
    #index    
    @QtCore.pyqtSignature("int")
    def on_slider_flexMCPindex_sliderMoved(self,value):
        joint = 14
        self.text_flexMCPindex.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdMCPindex_sliderMoved(self,value):
        joint = 15
        self.text_abdMCPindex.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
            
    @QtCore.pyqtSignature("int")
    def on_slider_flexPIPindex_sliderMoved(self,value):
        joint = 13
        self.text_flexPIPindex.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    @QtCore.pyqtSignature("int")
    def on_slider_flexDIPindex_sliderMoved(self,value):
        joint = 12
        self.text_flexDIPindex.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    #middle    
    @QtCore.pyqtSignature("int")
    def on_slider_flexMCPmiddle_sliderMoved(self,value):
        joint = 19
        self.text_flexMCPmiddle.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdMCPmiddle_sliderMoved(self,value):
        joint = 20
        self.text_abdMCPmiddle.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
            
    @QtCore.pyqtSignature("int")
    def on_slider_flexPIPmiddle_sliderMoved(self,value):
        joint = 18
        self.text_flexPIPmiddle.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    @QtCore.pyqtSignature("int")
    def on_slider_flexDIPmiddle_sliderMoved(self,value):
        joint = 17
        self.text_flexDIPmiddle.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_flexCMCring_sliderMoved(self,value):
        joint = 26
        self.text_flexCMCring.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
         
    @QtCore.pyqtSignature("int")
    def on_slider_flexMCPring_sliderMoved(self,value):
        joint = 24
        self.text_flexMCPring.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdMCPring_sliderMoved(self,value):
        joint = 25
        self.text_abdMCPring.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
            
    @QtCore.pyqtSignature("int")
    def on_slider_flexPIPring_sliderMoved(self,value):
        joint = 23
        self.text_flexPIPring.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
    
    @QtCore.pyqtSignature("int")
    def on_slider_flexDIPring_sliderMoved(self,value):
        joint = 22
        self.text_flexDIPring.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    #small
    @QtCore.pyqtSignature("int")
    def on_slider_flexCMCsmall_sliderMoved(self,value):
        joint = 31
        self.text_flexCMCsmall.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
         
    @QtCore.pyqtSignature("int")
    def on_slider_flexMCPsmall_sliderMoved(self,value):
        joint = 29
        self.text_flexMCPsmall.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))

    @QtCore.pyqtSignature("int")
    def on_slider_abdMCPsmall_sliderMoved(self,value):
        joint = 30
        self.text_abdMCPsmall.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
            
    @QtCore.pyqtSignature("int")
    def on_slider_flexPIPsmall_sliderMoved(self,value):
        joint = 28
        self.text_flexPIPsmall.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))
        
    @QtCore.pyqtSignature("int")
    def on_slider_flexDIPsmall_sliderMoved(self,value):
        joint = 27
        self.text_flexDIPsmall.setText(str(value))
        self.SendToOR("SetPosture",str(value)+' '+str(joint))    

    @QtCore.pyqtSignature("int")
    def on_ckBThumb_stateChanged(self,state):
        self.graspFingers[0]=state/2
        
    @QtCore.pyqtSignature("int")
    def on_ckBIndex_stateChanged(self,state):
        self.graspFingers[1]=state/2
        
    @QtCore.pyqtSignature("int")
    def on_ckBMiddle_stateChanged(self,state):
        self.graspFingers[2]=state/2
        
    @QtCore.pyqtSignature("int")
    def on_ckBRing_stateChanged(self,state):
        self.graspFingers[3]=state/2
        print self.graspFingers
        
    @QtCore.pyqtSignature("int")
    def on_ckBSmall_stateChanged(self,state):
        self.graspFingers[4]=state/2
        
    @QtCore.pyqtSignature("")
    def on_pbSaveInitial_clicked(self):
        print "Saving Initial Posture"        
        file=str(self.lEsaveInitial.text())
        if not file.endswith('.m'):
            file+='.m'
        path = self.umano_path+'env/postures/Humano/Initial/'+file
        text=str(self.tePosture.toPlainText())
        f = open (path, "w")
        f.write(text)
        f.close()
        self.LoadComboBoxes()

    @QtCore.pyqtSignature("")
    def on_pbRSaveInitial_clicked(self):
        print "Saving Initial Posture"  
        self.SendToOR("GetRobotPosture",callback=self.saveInitial)            
        
    @QtCore.pyqtSignature("")
    def on_pbSaveEnd_clicked(self):
        print "Saving End Posture"        
        file=str(self.lEsaveEnd.text())
        if not file.endswith('.m'):
            file+='.m'
        path = self.umano_path+'env/postures/Humano/End/'+file
        text=str(self.tePosture.toPlainText())
        f = open (path, "w")
        f.write(text)
        f.close()
        self.LoadComboBoxes()     
        
    @QtCore.pyqtSignature("")
    def on_pbGen_clicked(self):       
        
        hands=self.lWHands.selectedItems()
        hands=map(str,map(QtGui.QListWidgetItem.text, hands))        
        objects=self.lWObjects.selectedItems()
        objects=map(str,map(QtGui.QListWidgetItem.text, objects))        
        standoffs=self.lEStandoffs.text()
        maxgrasps=self.lEGrasps.text()
        if len(standoffs)==0:  
            standoffs=None
        else:                
            standoffs=standoffs.split(',')
            standoffs=map(float,map(str,standoffs))
                    
        rolls=self.lERolls.text()
        if len(rolls)==0:
            rolls=None
        else:
            rolls=rolls.split(',')
            rolls=map(float,map(str,rolls))        
        boxdelta=self.lEDelta.text()        
        normalanglerange=self.lEAngle.text()        
       
        
        if len(boxdelta)==0:
            boxdelta=None
        else:
            boxdelta=float(boxdelta)
        if len(maxgrasps)==0:
            maxgrasps=None
        else:
            maxgrasps=int(maxgrasps)
        if len(normalanglerange)==0:
            normalanglerange=None
        else:
            normalanglerange=pi/float(normalanglerange)
        if len(hands)==0  or hands[0]=='All':
            hands=[]
            count=self.lWHands.count()
            for i in range(1,count):
                hands.append(str(self.lWHands.item(i).text()))
        if len(objects)==0 or objects[0]=='All':
            objects=[]
            count=self.lWObjects.count()
            for i in range(1,count):
                objects.append(str(self.lWObjects.item(i).text()))
        experiment = str(self.lEGrasps_2.text())
        params=[hands, objects, standoffs, rolls, boxdelta, normalanglerange, maxgrasps, experiment]
          
        self.SendToOR("GenGrasps", params)
             
               
    @QtCore.pyqtSignature("")
    def on_pbBenchEval1_clicked(self):   
        
        #Load Hand
        envName = str(self.cBBenchHand1.currentText())
        if envName=="HumanHand":
           self.on_pbLoadArm_clicked()
        else:
           if "shadow" in envName:
               path = path_hand+envName+".dae"
           else:
               path = path_hand+envName+".robot.xml"
           print "Load hand"
           
           self.SendToOR("LoadHand",path)
        
        #Load Objects
        self.hdf5file.close()
        self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+envName+".hdf5", "a")
        self.handName=envName
        self.objects=self.hdf5file.keys()
        self.obj_pos=0        
        self.init_handeval=False
        self.accumulate=[0,0,0,0,0,0,0]
        self.cont=[0,0,0,0,0,0,0]
        self.benchmarking()
        
    def benchmarking(self):
        if self.obj_pos>=len(self.objects):
            print "hand eval finished"
            return True        
        object=self.objects[self.obj_pos]
        self.SendToOR("LoadObject", object)
        self.obj_grp=self.hdf5file[object]
        self.grasps_count=len(self.obj_grp['grasps'])             
        #LoadPostures
        self.ini=time.time()
        self.pos=0        
        self.values=[]
        posture=self.obj_grp["grasps"][self.pos]
        self.posture_num=0
        self.SendToOR("LoadPosture", posture, self.BenchGrasp)
        
            
    def BenchGrasp(self, params):
        if self.handName=="HumanHand":
            self.SendToOR("HumanGrasp", None, self.BenchMeasure)
        else:
            self.SendToOR("Grasp", None, self.BenchMeasure)
    
    def BenchMeasure(self, params):
        if params!=None:
            self.SendToOR("QM", params, self.BenchResults)
        else:
            print "Bad Grasp"
            self.BenchLoad()
            
    def BenchResults(self,params):        
        values=[params[2],params[3],params[4],params[5],params[7],params[8],params[9]]
        results=[]
        for i in range(7):
            if values[i]!=None:
                self.cont[i]+=1
                self.accumulate[i]+=values[i]
            if self.cont[i]==0:
                results.append(0)
            else:
                results.append((self.accumulate[i]/self.cont[i])*100/self.QC[i])
            
            if results[i]>200:
                self.labels_1[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_1[i].setStyleSheet(self.style2)
                self.progress_1[i].setValue(self.progress_1[i].minimum())
            elif results[i]>150:
                self.labels_1[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_1[i].setStyleSheet(self.style2)
                self.progress_1[i].setMinimum(100)
                self.progress_1[i].setMaximum(200)
                self.progress_1[i].setValue(300-results[i])
                #print self.progress_1[i].styleSheet()                
            elif results[i]>100:
                self.labels_1[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_1[i].setStyleSheet(self.style1)
                self.progress_1[i].setMinimum(50)
                self.progress_1[i].setMaximum(150)
                self.progress_1[i].setValue(200-results[i])
            else:
                self.progress_1[i].setStyleSheet(self.style0)
                self.labels_1[i].setText(str("%.2f"%(results[i]))+'%')
                self.progress_1[i].setMinimum(0)
                self.progress_1[i].setMaximum(100)
                self.progress_1[i].setValue(100-results[i])
        self.BenchLoad()

        
    def BenchLoad(self):        
        #time.sleep(0.1)
        self.pos+=1
        if self.pos>=self.grasps_count:
            print "Object poses ended"            
            self.obj_pos+=1            
            self.benchmarking()
            return True
        posture=self.obj_grp["grasps"][self.pos]        
        self.SendToOR("LoadPosture", posture, self.BenchGrasp)
        

    
    @QtCore.pyqtSignature("")
    def on_pbBenchEval2_clicked(self):  
        #Load Hand
        envName = str(self.cBBenchHand2.currentText())
        if envName=="HumanHand":
           self.on_pbLoadArm_clicked()
        else:
           if "shadow" in envName:
               path = path_hand+envName+".dae"
           else:
               path = path_hand+envName+".robot.xml"
           print "Load hand"
           
           self.SendToOR("LoadHand",path)
        
        #Load Objects
        self.hdf5file.close()
        self.hdf5file=h5py.File(path_experiments+self.experiment+"/"+envName+".hdf5", "a")
        self.handName=envName
        self.objects=self.hdf5file.keys()
        self.obj_pos2=0        
        self.init_handeval=False
        self.accumulate=[0,0,0,0,0,0,0]
        self.cont=[0,0,0,0,0,0,0]
        self.benchmarking2()
    
    def benchmarking2(self):
        if self.obj_pos2>=len(self.objects):
            print "hand eval finished"
            return True        
        object=self.objects[self.obj_pos2]
        self.SendToOR("LoadObject", object)
        self.obj_grp=self.hdf5file[object]
        self.grasps_count2=len(self.obj_grp['grasps'])             
        #LoadPostures
        self.ini=time.time()
        self.pos2=0        
        self.values=[]
        posture=self.obj_grp["grasps"][self.pos2]
        self.posture_num2=0
        self.SendToOR("LoadPosture", posture, self.BenchGrasp2)
            
    def BenchGrasp2(self, params):
        if self.handName=="HumanHand":
            self.SendToOR("HumanGrasp", None, self.BenchMeasure2)
        else:
            self.SendToOR("Grasp", None, self.BenchMeasure2)
    
    def BenchMeasure2(self, params):
        self.SendToOR("QM", params, self.BenchResults2)
        
    def BenchResults2(self,params):
        values=[params[2],params[3],params[4],params[5],params[7],params[8],params[9]]
        results=[]
        for i in range(7):          
            if values[i]!=None:
                self.cont[i]+=1
                self.accumulate[i]+=values[i]
            if self.cont[i]==0:
                results.append(0)
            else:
                results.append((self.accumulate[i]/self.cont[i])*100/self.QC[i])
            
            if results[i]>200:
                self.labels_2[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_2[i].setValue(self.progress_2[i].minimum())
                self.progress_2[i].setStyleSheet(self.style2)
            elif results[i]>150:
                self.labels_2[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_2[i].setStyleSheet(self.style2)
                self.progress_2[i].setMinimum(100)
                self.progress_2[i].setMaximum(200)
                self.progress_2[i].setValue(300-results[i])
                #print self.progress_1[i].styleSheet()                
            elif results[i]>100:
                self.labels_2[i].setText(str("+%.2f"%(results[i]-100))+'%')
                self.progress_2[i].setStyleSheet(self.style1)
                self.progress_2[i].setMinimum(50)
                self.progress_2[i].setMaximum(150)
                self.progress_2[i].setValue(200-results[i])
            else:
                self.progress_2[i].setStyleSheet(self.style0)
                self.labels_2[i].setText(str("%.2f"%(results[i]))+'%')
                self.progress_2[i].setMinimum(0)
                self.progress_2[i].setMaximum(100)
                self.progress_2[i].setValue(100-results[i])

        self.BenchLoad2()
    
    def BenchLoad2(self):
        self.pos2+=1
        if self.pos2>=self.grasps_count2:
            print "Object poses ended"            
            self.obj_pos2+=1            
            self.benchmarking2()
            return True
        posture=self.obj_grp["grasps"][self.pos2]        
        self.SendToOR("LoadPosture", posture, self.BenchGrasp2)
    
        
    def loadAll(self, params=None):  
        
        self.saveMeasure(params)
        self.showStatistics(params)
        
        self.fin=time.time()
        self.updateTeOutput("Loading next grasp")        
        self.posture_num+=1
        self.pos+=1
        if self.pos>=len(self.obj_grp['grasps']):
            self.updateTeOutput("Evaluation finished")
            self.updateTeOutput("Execution Time: " + str("%.2f"%(self.fin-self.ini)) + " seconds")
        else:
            posture=self.obj_grp["grasps"][self.pos]
            self.SendToOR("LoadPosture", posture, self.graspAll)
        
    def ReadPosture(self, file=None):        
        if file==None:            
            if not hasattr(self, 'f'):
                posturePath = str(self.umano_path+'env/postures/'+self.hand+'/Initial/')
                postureFile = self.postures[self.actual]
                self.f=open(posturePath+postureFile+".m", 'r')
                aux=postureFile.split('_')
                meshName=aux[0]+'_5k.wrl'       
                print meshName         
                self.SendToOR("LoadObject",meshName)  

            value=self.f.readline()
                
            if value=="#\n":
                print "fin de fichero"
                self.actual+=1
                if self.actual>=len(self.postures):
                    print "evaluados todos los agarres"
                    self.f.close()
                    delattr(self, 'f')
                    return -1,-1
                posturePath = str(self.umano_path+'env/postures/'+self.hand+'/Initial/')
                postureFile = self.postures[self.actual]
                self.f.close()
                self.f=open(posturePath+postureFile+".m", 'r')
                aux=postureFile.split('_')
                meshName=aux[0]+'_5k.wrl'      
                self.SendToOR("LoadObject",meshName) 
                time.sleep(0.5) 
                value=self.f.readline()
            
        else:
            self.f=open(file, 'r')
            value=self.f.readline()
            
        res=[]
        res.append(value)
        while value!="#\n":
            value=self.f.readline()
            res.append(value)
        self.pos=res[0].split()[0]
        res=res[1:-1]
        for i in range(len(res)):
            res[i]=res[i].split()
        for i in range(len(res)):
            for j in range(len(res[i])):
                res[i][j]=float(res[i][j])
        self.dof=res[0]        
        self.transform=res[1:]       
        return self.dof, self.transform
        
    def ModifyPosture(self):
        transform_new=copy.deepcopy(self.transform)
        dof_new=self.dof
        step=0.002
        rot=0.02
        if self.j_sen<=5:
            transform_new[0][3]=(transform_new[0][3])+step*(self.j_sen)
        elif self.j_sen<=10:
            transform_new[0][3]=(transform_new[0][3])-step*(self.j_sen-5)
        elif self.j_sen<=15:
            transform_new[1][3]=(transform_new[1][3])+step*(self.j_sen-10)
        elif self.j_sen<=20:
            transform_new[1][3]=(transform_new[1][3])-step*(self.j_sen-15)    
        elif self.j_sen<=25:
            transform_new[2][3]=(transform_new[2][3])+step*(self.j_sen-20)
        elif self.j_sen<=30:
            transform_new[2][3]=(transform_new[2][3])-step*(self.j_sen-25)
        elif self.j_sen<=35:
            alfa=-(self.j_sen-30)*rot
            rot_x=matrixFromAxisAngle([alfa,0,0])
            transform_new=numpy.dot(rot_x, transform_new)
        elif self.j_sen<=40:
            alfa=(self.j_sen-35)*rot
            rot_x=matrixFromAxisAngle([alfa,0,0])
            transform_new=numpy.dot(rot_x, transform_new)
        elif self.j_sen<=45:
            alfa=-(self.j_sen-40)*rot
            rot_y=matrixFromAxisAngle([0,alfa,0])
            transform_new=numpy.dot(rot_y, transform_new)
        elif self.j_sen<=50:
            alfa=(self.j_sen-45)*rot
            rot_y=matrixFromAxisAngle([0,alfa,0])
            transform_new=numpy.dot(rot_y, transform_new)
        elif self.j_sen<=55:
            alfa=-(self.j_sen-50)*rot
            rot_z=matrixFromAxisAngle([0,0,alfa])
            transform_new=numpy.dot(rot_z, transform_new)
        elif self.j_sen<=60:
            alfa=(self.j_sen-55)*rot
            rot_z=matrixFromAxisAngle([0,0,alfa])
            transform_new=numpy.dot(rot_z, transform_new)
        self.j_sen+=1            
        return dof_new, transform_new
            
        
    def graspAll(self, params):
        self.updateTeOutput("calling grasp")
        if self.handName=="HumanHand":
            self.SendToOR("HumanGrasp", None, self.measureAll)
        else:
            self.SendToOR("Grasp", None, self.measureAll)
    
    def testAll(self, params=None):       
        self.fin=time.time()
        print "Execution Time: " + str("%.2f"%(self.fin-self.ini)) + " seconds"        
        if self.lERSaveInitial.text()=='':
            file='../results/output.txt'
        else:
            file='../results/'+str(self.lERSaveInitial.text())
        with open(file, 'a') as f: 
            f.write(self.postures[self.actual]+"_"+self.pos)
            f.write('\t')           
            if params>0:
               f.write("1")
            else:
               f.write("0")
            f.write('\n')
        f.close()
        self.ini=time.time()
        (dof, transform) = self.ReadPosture()
        if self.actual>=len(self.postures):
            print "Fin de la simulacion de agarres"
            #delattr(self, 'f')
            return True
        self.SendToOR("LoadPosture", (dof, transform), self.testGrasp)
    
    def testGrasp(self, params=None):
        self.SendToOR("testGrasp", None, self.testAll)
        

     
    def measure(self, params):
        #print params
        if params!=0.0:
            contactsVector=params[0]
            dits=params[1]
            links=params[2]
            self.SendToOR("QM", params, self.saveMeasure)
             

    def saveMeasure(self, params):
        
        self.fin=time.time()        
        posture_num=self.posture_num
        params[params<0]=-1
        params=hstack(params)
        
        path = path_experiments+self.experiment+"/"
        self.updateTeOutput("Saving measures in hdf5 file")
        
        if 'quality' not in self.obj_grp.keys():
            quality_ds=self.obj_grp.create_dataset('quality', (len(self.obj_grp['grasps']),10), chunks=True, dtype=numpy.float64, compression="gzip", compression_opts=9, maxshape=(None, 10))
        else:
            try:
                quality_ds=self.obj_grp['quality']
                if len(self.obj_grp['grasps'])>len(quality_ds):
                     self.obj_grp['quality'].resize(size=(len(self.obj_grp['grasps']), 10))
            except Exception as e:
                print "Error getting actual quality dataset, recreating"                
                self.obj_grp.__delitem__('quality')
                quality_ds=self.obj_grp.create_dataset('quality', (len(self.obj_grp['grasps']),10), chunks=True, dtype=numpy.float64, compression="gzip", compression_opts=9, maxshape=(None, 10))
                         
        quality_ds[posture_num]=params
        
        self.hdf5file.close()        
        self.hdf5file=h5py.File(path+self.handName+".hdf5", 'a')
        self.obj_grp=self.hdf5file[self.meshName]
        
        self.updateMeasures(params)        

        self.updateTeOutput("Execution Time: " + str("%.2f"%(self.fin-self.ini)) + " seconds")

    def measureAll(self, params):
        if params!=0.0:
            contactsVector=params[0]
            dits=params[1]
            links=params[2]
            
            self.SendToOR("QM", params, self.loadAll)            
    
    def RQM(self, params):        
        contactsVector=params[0]
        dits=params[1]
        links=params[2]
        self.SendToOR("QM", params, self.updateMeasures)
    
    def RQM_Matlab(self, params):        
        contactsVector=params[0]
        dits=params[1]
        links=params[2]
        self.SendToServer("___QM___",params, callback=self.updateMeasures)
         
                
    def updateMeasures(self, text):
        if len(text)==12:                
            self.textA3.setText(str("%.4f"%text[2][0]))
            self.textB1.setText(str("%.4f"%text[3][0]))
            self.textC1.setText(str("%.4f"%text[6][0]))
            self.textD1.setText(str("%.4f"%text[8][0]))
            self.textD2.setText(str("%.4f"%text[9][0]))
            self.textE1.setText(str("%.4f"%text[10][0]))
            self.textClas.setText(str("%.4f"%text[11][0]))  
            self.grview = self.graphicsMeasures          
        else:
            self.textA1G.setText(str("%.4f"%(text[0])))
            self.textB1G.setText(str("%.4f"%(text[3])))
            self.textB2G.setText(str("%.4f"%(text[4])))
            self.textB3G.setText(str("%.4f"%(text[5])))
            self.textC2G.setText(str("%.4f"%(text[7])))
            self.textD1G.setText(str("%.4f"%(text[8])))
            self.textD2G.setText(str("%.4f"%(text[9])))
        
    def cleanValues(self):
        self.textA1G.setText("")
        self.textB1G.setText("")
        self.textB2G.setText("")
        self.textB3G.setText("")
        self.textC2G.setText("")
        self.textD1G.setText("")
        self.textD2G.setText("")
        
        self.textA1av.setText("")
        self.textB1av.setText("")
        self.textB2av.setText("")
        self.textB3av.setText("")
        self.textC2av.setText("")
        self.textD1av.setText("")
        self.textD2av.setText("")

        self.textA1std.setText("")
        self.textB1std.setText("")
        self.textB2std.setText("")
        self.textB3std.setText("")
        self.textC2std.setText("")
        self.textD1std.setText("")
        self.textD2std.setText("")
        
        
    def showStatistics(self, text):
        
        
        text[text==-1]=NaN
        text=hstack(text)
        self.values.append(text)
        text_av=nanmean(self.values, 0)
        
        self.textA1av.setText(str("%.4f"%(text_av[0])))
        self.textB1av.setText(str("%.4f"%(text_av[3])))
        self.textB2av.setText(str("%.4f"%(text_av[4])))
        self.textB3av.setText(str("%.4f"%(text_av[5])))
        self.textC2av.setText(str("%.4f"%(text_av[7])))
        self.textD1av.setText(str("%.4f"%(text_av[8])))
        self.textD2av.setText(str("%.4f"%(text_av[9])))

        text_std=nanstd(self.values, 0)
        self.textA1std.setText(str("%.4f"%(text_std[0])))
        self.textB1std.setText(str("%.4f"%(text_std[3])))
        self.textB2std.setText(str("%.4f"%(text_std[4])))
        self.textB3std.setText(str("%.4f"%(text_std[5])))
        self.textC2std.setText(str("%.4f"%(text_std[7])))
        self.textD1std.setText(str("%.4f"%(text_std[8])))
        self.textD2std.setText(str("%.4f"%(text_std[9])))
        
        
        
    def showMeasures(self, measures):
        self.textA3.setText(measures[0])
        self.textB1.setText(measures[1])
        self.textC1.setText(measures[2])
        self.textD1.setText(measures[3])
        self.textD2.setText(measures[4])
        self.textE1.setText(measures[5])
        self.textClas.setText(measures[6])
        
    def showGraph(self):
        self.doc = popplerqt4.Poppler.Document.load('Posture_Graph.pdf')
        self.doc.setRenderBackend(popplerqt4.Poppler.Document.RenderBackend(0))
        self.doc.setRenderHint(popplerqt4.Poppler.Document.RenderHint(3))        
        self.page = self.doc.page(0)         
        self.image = self.page.renderToImage()
        img=QtGui.QPixmap.fromImage(self.image)       
        img=img.scaled(490, 490, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.scene.addPixmap(img)
        self.grview.setScene(self.scene)  
        self.grview.show()   

    def clearWindow(self):
        
        #Clear QM
        self.textA3.setText("")
        self.textB1.setText("")
        self.textC1.setText("")
        self.textD1.setText("")
        self.textD2.setText("")
        self.textE1.setText("")
        self.textClas.setText("")
        self.scene.clear()      
        
        #Clear QM Robot
        self.textA3R.setText("")
        self.textB1R.setText("")
        self.textC1R.setText("")
        self.textD1R.setText("")
        self.textD2R.setText("")    
        
        #Disable Buttons
        self.pbLoadMarkers.setEnabled(False)
        self.pbLoadInitial.setEnabled(False)
        self.pbLoadEnd.setEnabled(False)
        self.pbGrasp.setEnabled(False)
        self.pbQM.setEnabled(False)
        
        #Disable Buttons Robot
        self.pbRLoadEnv.setEnabled(False)
        self.pbRLoadObject.setEnabled(False)
        self.pbRLoadMarkers.setEnabled(False)
        self.pbRInitialPosture.setEnabled(False)
        self.pbRGrasp.setEnabled(False)
        self.pbRQM.setEnabled(False)

        
    def updateSliders(self,text):

        values = text.split('\t')
        values.remove('posture(1,:)=[')
        values.remove('];\nposture(2,:)=[')
        values.remove('];\nposture(3,:)=[')
        values.remove('];\nposture(4,:)=[')
        values.remove('];\nposture(5,:)=[')
        values.remove('];\n\narm_posture(1,:) = [')
        values.remove('];\narm_posture(2,:) = [')
        values.remove('];\narm_posture(3,:) = [')
        
        for value in values:
            value = value.strip('\n')

        self.slider_fi_s.setValue(float(values[30]))
        self.text_fi_s.setText("%.2f" % float(values[30]))
        
        self.slider_theta_s.setValue(float(values[31]))
        self.text_theta_s.setText("%.2f" % float(values[31]))
        
        self.slider_psi_s.setValue(float(values[32]))
        self.text_psi_s.setText("%.2f" % float(values[32]))
        
        self.slider_psi_fe_e.setValue(float(values[33]))
        self.text_psi_fe_e.setText("%.2f" % float(values[33]))
        
        self.slider_psi_ps_e.setValue(float(values[34]))
        self.text_psi_ps_e.setText("%.2f" % float(values[34]))
        
        self.slider_psi_fe_w.setValue(float(values[36]))
        self.text_psi_fe_w.setText("%.2f" % float(values[36]))
        
        self.slider_psi_abd_w.setValue(float(values[37]))
        self.text_psi_abd_w.setText("%.2f" % float(values[37]))
        
        self.slider_flexCMCthumb.setValue(float(values[0]))
        self.text_flexCMCthumb.setText("%.2f" % float(values[0]))
        
        self.slider_abdCMCthumb.setValue(float(values[1]))
        self.text_abdCMCthumb.setText("%.2f" % float(values[1]))
        
        self.slider_flexMCPthumb.setValue(float(values[2]))
        self.text_flexMCPthumb.setText("%.2f" % float(values[2]))
        
        self.slider_abdMCPthumb.setValue(float(values[3]))
        self.text_abdMCPthumb.setText("%.2f" % float(values[3]))
        
        self.slider_flexIPthumb.setValue(float(values[4])) #5,6,7
        self.text_flexIPthumb.setText("%.2f" % float(values[4]))
        
        self.slider_flexMCPindex.setValue(float(values[8]))
        self.text_flexMCPindex.setText("%.2f" % float(values[8]))
        
        self.slider_abdMCPindex.setValue(float(values[9]))
        self.text_abdMCPindex.setText("%.2f" % float(values[9]))
        
        self.slider_flexPIPindex.setValue(float(values[10]))
        self.text_flexPIPindex.setText("%.2f" % float(values[10]))
        
        self.slider_flexDIPindex.setValue(float(values[11]))
        self.text_flexDIPindex.setText("%.2f" % float(values[11]))
       
        self.slider_flexMCPmiddle.setValue(float(values[14]))
        self.text_flexMCPmiddle.setText("%.2f" % float(values[14]))
        
        self.slider_abdMCPmiddle.setValue(float(values[15]))
        self.text_abdMCPmiddle.setText("%.2f" % float(values[15]))
        
        self.slider_flexPIPmiddle.setValue(float(values[16]))
        self.text_flexPIPmiddle.setText("%.2f" % float(values[16]))
        
        self.slider_flexDIPmiddle.setValue(float(values[17]))
        self.text_flexDIPmiddle.setText("%.2f" % float(values[17]))
        
        self.slider_flexCMCring.setValue(float(values[18]))
        self.text_flexCMCring.setText("%.2f" % float(values[18]))
        
        self.slider_flexMCPring.setValue(float(values[20]))
        self.text_flexMCPring.setText("%.2f" % float(values[20]))
        
        self.slider_abdMCPring.setValue(float(values[21]))
        self.text_abdMCPring.setText("%.2f" % float(values[21]))
        
        self.slider_flexPIPring.setValue(float(values[22]))
        self.text_flexPIPring.setText("%.2f" % float(values[22]))
        
        self.slider_flexDIPring.setValue(float(values[23]))
        self.text_flexDIPring.setText("%.2f" % float(values[23]))
        
        self.slider_flexCMCsmall.setValue(float(values[24]))
        self.text_flexCMCsmall.setText("%.2f" % float(values[24]))
        
        self.slider_flexMCPsmall.setValue(float(values[26]))
        self.text_flexMCPsmall.setText("%.2f" % float(values[26]))
        
        self.slider_abdMCPsmall.setValue(float(values[27]))
        self.text_abdMCPsmall.setText("%.2f" % float(values[27]))
        
        self.slider_flexPIPsmall.setValue(float(values[28]))
        self.text_flexPIPsmall.setText("%.2f" % float(values[28]))
        
        self.slider_flexDIPsmall.setValue(float(values[29]))
        self.text_flexDIPsmall.setText("%.2f" % float(values[29]))
    
    def updateTePosture(self,text):
        self.tePosture.setText(str(text))
        
    def saveInitial(self, text):
        file=str(self.lERSaveInitial.text())
        if not file.endswith('.m'):
            file+='.m'
        path = self.umano_path+'env/postures/'+self.hand+'/Initial/'+file
        f = open (path, "w")
        f.write(text)
        f.close()
        self.LoadComboBoxes()  
                  
    def updateTeOutput(self,text):
        content = str(text)
        content += "\n"
        content += str(self.teOutput.toPlainText())
        self.teOutput.setText(content)

    def CallbackOR(self,args):
        self.ButtonsUnlock()

    def SendToServer(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandlerServer.callback = callback
            self.CallbackHandlerServer.start()
            handleCallback=True
        self.pipeServer.send([command,args,handleCallback])

    def SendToOR(self,command,args=None,callback=None):
        handleCallback=False
        if callback:
            self.CallbackHandler.callback = callback
            self.CallbackHandler.start()
            handleCallback=True
        self.pipeOR.send([command,args,handleCallback])

    def HandleCallback(self,msg):
        if(len(msg) == 2):
            if(msg[0] is not None):
                try:
                    msg[0](msg[1][1])
                except Exception as e:
                    logger.error(str(e))
            else:
                self.updateTeOutput("ERROR: "+msg[1][0])
            return
        logger.error("ERROR in request format")
        
        
def main(env,options):
    "Main example code."
    
    global logger
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    logger = logging.getLogger('PyqtControl')
    lhandler =logging.StreamHandler(sys.stdout)
    lhandler.setFormatter(logging.Formatter("%(levelname)-10s:: %(filename)-20s - %(lineno)4d :: %(message)s"))
    logger.setLevel(logging.INFO)
    logger.addHandler(lhandler)
    server = Server()
    

from optparse import OptionParser

def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(description='Pyqt example to demonstrate how openrave elements can be controlled by a qt-gui.', usage='openrave.py --example qtserverprocess [options]')
    (options, leftargs) = parser.parse_args(args=args)
    main(None,options)

if __name__ == "__main__":
    run()
