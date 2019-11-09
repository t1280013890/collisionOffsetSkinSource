#pragma once
const char* pythonScript1 = "import PySide2.QtGui as QtGui\n\
import PySide2.QtWidgets as QtWidgets\n\
import PySide2.QtCore as QtCore\n\
from shiboken2 import wrapInstance\n\
import maya.OpenMayaUI as OpenMayaUI\n\
import maya.OpenMaya as OpenMaya\n\
import maya.OpenMayaAnim as OMA\n\
import maya.cmds as cmds\n\
class fixDoubleValidator(QtGui.QDoubleValidator):\n\
    def __init__(self,min,max,decimals):\n\
        super(fixDoubleValidator,self).__init__()\n\
        self.setBottom(min)\n\
        self.setTop(max)\n\
        self.setDecimals(decimals)\n\
        self.min = min\n\
        self.max =max\n\
    def validate(self,input,pos):\n\
        state = super(fixDoubleValidator,self).validate(input,pos)\n\
        if state[0] == QtGui.QValidator.Intermediate:\n\
            num = 0\n\
            try:\n\
                num = float(input)\n\
            except:\n\
                return state\n\
            if num<0:\n\
                if num<self.min:\n\
                    return (QtGui.QValidator.Invalid,state[1],state[2])\n\
                return state\n\
            else:\n\
                if num>self.max:\n\
                    return (QtGui.QValidator.Invalid,state[1],state[2])\n\
                return state\n\
        return state\n\
class parameterItem(QtWidgets.QWidget):\n\
    value = 0\n\
    def __init__(self,parentPtr,name,min,max,showSlide=True,parent=None):\n\
        self.parentPtr = parentPtr\n\
        self.attrName = name\n\
        self.min = float(min)\n\
        self.max = float(max)\n\
        self.showSlide = showSlide\n\
        self.attrConnected = True\n\
        self.value = cmds.getAttr('{nodeName}.influenceInfo[{index}].{attrName}'.format(\n\
            nodeName = self.parentPtr.nodeName,index=self.parentPtr.indexList[-1],attrName=self.attrName))\n\
        if showSlide:\n\
            self.slideMax=self.max\n\
        else:\n\
            self.slideMax=self.value*2\n\
            if self.slideMax<1:\n\
                self.slideMax=1\n\
        super(parameterItem,self).__init__(parent)\n\
        mainLayout = QtWidgets.QHBoxLayout()\n\
        mainLayout.setMargin(0)\n\
        self.setLayout(mainLayout)\n\
        lable = QtWidgets.QLabel(name)\n\
        lable.setAlignment(QtCore.Qt.AlignRight)\n\
        lable.setFixedWidth(135)\n\
        self.numberLine = QtWidgets.QLineEdit()\n\
        self.numberLine.setFixedWidth(70)\n\
        self.numberLine.setValidator(fixDoubleValidator(min,max,3))\n\
        mainLayout.addWidget(lable)\n\
        mainLayout.addWidget(self.numberLine)\n\
        self.numberLine.editingFinished.connect(self.on_editingFinished)\n\
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)\n\
        self.slider.setFixedWidth(200)\n\
        self.slider.setMaximum(self.slideMax*1000)\n\
        self.slider.setMinimum(min*1000)\n\
        mainLayout.addWidget(self.slider)\n\
        self.slider.sliderMoved.connect(self.on_sliderMoved)\n\
        self.slider.valueChanged.connect(self.on_valueChanged)\n\
        self.refreshValue()\n\
    def on_editingFinished(self):\n\
        try:\n\
            self.value = float(self.numberLine.text())\n\
            if self.value>self.slideMax:\n\
                self.slideMax=self.value\n\
                self.slider.setMaximum(self.slideMax*1000)\n\
            self.slider.setValue(self.value*1000)\n\
        except:\n\
            pass\n\
    def on_sliderMoved(self,p):\n\
        self.value = p/1000.0\n\
        self.numberLine.setText('%.3f'%self.value)\n\
    def on_valueChanged(self,p):\n\
        if self.attrConnected:\n\
            for index in self.parentPtr.indexList:\n\
                cmds.setAttr('{nodeName}.influenceInfo[{index}].{attrName}'.format(\n\
                    nodeName=self.parentPtr.nodeName,index=index,attrName=self.attrName),self.value)\n\
    def refreshValue(self):\n\
        self.value = cmds.getAttr('{nodeName}.influenceInfo[{index}].{attrName}'.format(\n\
            nodeName=self.parentPtr.nodeName, index=self.parentPtr.indexList[-1], attrName=self.attrName))\n\
        self.numberLine.setText('%.3f'%self.value)\n\
        self.attrConnected = False\n\
        if(self.value>self.slideMax):\n\
            self.slidMax = self.valur\n\
            self.slider.setMaximum(self.slideMax*1000)\n\
        self.slider.setValue(self.value*1000)\n\
        self.attrConnected=True\n\
";

const char* pythonScript2 = "class skinParameterWidget(QtWidgets.QWidget):\n\
    window = None\n\
    point = QtCore.QPoint()\n\
    isDrag = False\n\
    windowPosition = None\n\
    comBoxText = None\n\
    vectorDic={'x':OpenMaya.MVector(1,0,0),\n\
               'y':OpenMaya.MVector(0,1,0),\n\
               'z':OpenMaya.MVector(0,0,1),\n\
               '-x':OpenMaya.MVector(-1,0,0),\n\
               '-y':OpenMaya.MVector(0,-1,0),\n\
               '-z':OpenMaya.MVector(0,0,-1),}\n\
    def __init__(self,nodeInfList):\n\
        self.isLinearEnable = True\n\
        self.isAxialEnable = True\n\
        self.nodeInfList = nodeInfList\n\
        self.nodeName = nodeInfList[0][0]\n\
        self.indexList = nodeInfList[0][2]\n\
        mainwindow = OpenMayaUI.MQtUtil.mainWindow()\n\
        qtmainwindow = wrapInstance(long(mainwindow),QtWidgets.QWidget)\n\
        super(skinParameterWidget,self).__init__(qtmainwindow)\n\
        mainLayout = QtWidgets.QVBoxLayout()\n\
        mainLayout.setSpacing(0)\n\
\n\
        self.jointLengthC = parameterItem(self,'jointLengthC',0,0xFFFF,False)\n\
        self.jointLengthP = parameterItem(self,'jointLengthP',0,0xFFF,False)\n\
        self.twistCutWeightP = parameterItem(self,'twistCutWeightP',0,1)\n\
        self.twistCutWeightC = parameterItem(self,'twistCutWeightC',0,1)\n\
        self.outsideCutProportion = parameterItem(self,'outsideCutProportion',0,0xFFFF,False)\n\
        self.collisionHeight = parameterItem(self,'collisionHeight',0,0xFFFF,False)\n\
        self.collisionDeep = parameterItem(self,'collisionDeep',0,1)\n\
        self.collisionOffset = parameterItem(self,'collisionOffset',0,1)\n\
        self.startAngleCos = parameterItem(self,'startAngleCos',-1,1)\n\
        self.endAngleCos = parameterItem(self,'endAngleCos',-1,1)\n\
        self.collisionSmoothP = parameterItem(self,'collisionSmoothP',0,1)\n\
        self.collisionSmoothC = parameterItem(self,'collisionSmoothC',0,1)\n\
        self.sideSmoothP = parameterItem(self,'sideSmoothP',0,1)\n\
        self.sideSmoothC = parameterItem(self,'sideSmoothC',0,1)\n\
        self.outsideExtendWeightP = parameterItem(self,'outsideExtendWeightP',0,1)\n\
        self.outsideExtendWeightC = parameterItem(self,'outsideExtendWeightC',0,1)\n\
        self.outsideHeight = parameterItem(self,'outsideHeight',-1,2)\n\
\n\
        hlayout0 = QtWidgets.QHBoxLayout()\n\
        hlayout0.setSpacing(2)\n\
        hlayout0.setMargin(2)\n\
        spaceLabel = QtWidgets.QLabel()\n\
        spaceLabel.setFixedWidth(135)\n\
        self.isLinearCheck = QtWidgets.QCheckBox()\n\
        self.isLinearCheck.setFixedWidth(15)\n\
        self.refreshIsLinearCheck()\n\
        isLinearLabel = QtWidgets.QLabel('isLinear')\n\
        isLinearLabel.setFixedWidth(50)\n\
        hlayout0.addWidget(spaceLabel)\n\
        hlayout0.addWidget(self.isLinearCheck)\n\
        hlayout0.addWidget(isLinearLabel)\n\
        hlayout0.addStretch()\n\
\n\
        hlayout1 = QtWidgets.QHBoxLayout()\n\
        hlayout1.setSpacing(2)\n\
        hlayout1.setMargin(2)\n\
        parentOffsetLabel = QtWidgets.QLabel(u'父骨骼偏移')\n\
        parentOffsetLabel.setFixedWidth(135)\n\
        parentOffsetLabel.setAlignment(QtCore.Qt.AlignRight)\n\
        alignParentButton = QtWidgets.QPushButton(u'归零')\n\
        alignChildButton = QtWidgets.QPushButton(u'对齐到子骨骼')\n\
        hlayout1.addWidget(parentOffsetLabel)\n\
        hlayout1.addWidget(alignParentButton)\n\
        hlayout1.addWidget(alignChildButton)\n\
\n\
        hlayout2 = QtWidgets.QHBoxLayout()\n\
        hlayout2.setMargin(1)\n\
        meshLabel = QtWidgets.QLabel('mesh')\n\
        meshLabel.setAlignment(QtCore.Qt.AlignRight)\n\
        meshLabel.setFixedWidth(135)\n\
        self.meshComb = QtWidgets.QComboBox()\n\
        self.meshComb.addItems([nodeInf[1] for nodeInf in nodeInfList])\n\
        hlayout2.addWidget(meshLabel)\n\
        hlayout2.addWidget(self.meshComb)\n\
\n\
        hlayout3 = QtWidgets.QHBoxLayout()\n\
        hlayout3.setMargin(1)\n\
        axialLabel = QtWidgets.QLabel(u'影响物轴向')\n\
        axialLabel.setFixedWidth(135)\n\
        axialLabel.setAlignment(QtCore.Qt.AlignRight)\n\
        frontLabel =  QtWidgets.QLabel(u'前方向轴')\n\
        frontLabel.setAlignment(QtCore.Qt.AlignRight)\n\
        self.frontCombox = QtWidgets.QComboBox()\n\
        self.frontCombox.addItems(['x','y','z','-x','-y','-z'])\n\
        self.frontCombox.setFixedWidth(60)\n\
        upLabel = QtWidgets.QLabel(u'上方向轴')\n\
        upLabel.setAlignment(QtCore.Qt.AlignRight)\n\
        self.upCombox = QtWidgets.QComboBox()\n\
        self.upCombox.setFixedWidth(60)\n\
        self.refreshAxialCombox()\n\
        hlayout3.addWidget(axialLabel)\n\
        hlayout3.addWidget(frontLabel)\n\
        hlayout3.addWidget(self.frontCombox)\n\
        hlayout3.addWidget(upLabel)\n\
        hlayout3.addWidget(self.upCombox)\n\
\n\
        self.setLayout(mainLayout)\n\
        mainLayout.addLayout(hlayout0)\n\
        mainLayout.addLayout(hlayout1)\n\
        mainLayout.addLayout(hlayout2)\n\
        mainLayout.addLayout(hlayout3)\n\
        mainLayout.addWidget(self.jointLengthP)\n\
        mainLayout.addWidget(self.jointLengthC)\n\
        mainLayout.addWidget(self.twistCutWeightP)\n\
        mainLayout.addWidget(self.twistCutWeightC)\n\
        mainLayout.addWidget(self.outsideCutProportion)\n\
        mainLayout.addWidget(self.outsideExtendWeightP)\n\
        mainLayout.addWidget(self.outsideExtendWeightC)\n\
        mainLayout.addWidget(QtWidgets.QLabel('--------------------------'))\n\
        mainLayout.addWidget(self.startAngleCos)\n\
        mainLayout.addWidget(self.endAngleCos)\n\
        mainLayout.addWidget(QtWidgets.QLabel('--------------------------'))\n\
        mainLayout.addWidget(self.collisionHeight)\n\
        mainLayout.addWidget(self.collisionDeep)\n\
        mainLayout.addWidget(self.collisionOffset)\n\
        mainLayout.addWidget(self.outsideHeight)\n\
        mainLayout.addWidget(self.collisionSmoothP)\n\
        mainLayout.addWidget(self.collisionSmoothC)\n\
        mainLayout.addWidget(self.sideSmoothP)\n\
        mainLayout.addWidget(self.sideSmoothC)\n\
        if skinParameterWidget.windowPosition != None:\n\
            self.move(skinParameterWidget.windowPosition)\n\
        if skinParameterWidget.comBoxText != None:\n\
            self.meshComb.setCurrentText(skinParameterWidget.comBoxText)\n\
            self.on_meshComb(self.meshComb.currentIndex())\n\
        skinParameterWidget.comBoxText = self.meshComb.currentText()\n\
        self.isLinearCheck.toggled.connect(self.on_isLinearCheck)\n\
        alignParentButton.clicked.connect(self.on_alignParentButton)\n\
        alignChildButton.clicked.connect(self.on_alignChildButton)\n\
        self.meshComb.currentIndexChanged.connect(self.on_meshComb)\n\
        self.frontCombox.currentTextChanged.connect(self.on_frontComb)\n\
        self.upCombox.currentTextChanged.connect(self.on_upComb)\n\
        self.setWindowFlags(self.windowFlags()|QtCore.Qt.Window|QtCore.Qt.FramelessWindowHint)\n\
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground,True)\n\
    def on_isLinearCheck(self,check):\n\
        if self.isLinearEnable:\n\
            for nodeName,null,indexList in self.nodeInfList:\n\
                for index in indexList:\n\
                    cmds.setAttr('{nodeName}.influenceInfo[{index}].isLinear'.format(\n\
                        nodeName = nodeName,index=index),check)\n\
    def refreshIsLinearCheck(self):\n\
        self.isLinearEnable = False\n\
        isLinear = cmds.getAttr('{nodeName}.influenceInfo[{index}].isLinear'.format(nodeName=self.nodeName,index=self.indexList[0]))\n\
        self.isLinearCheck.setChecked(isLinear)\n\
        self.isLinearEnable=True\n\
    def refreshAxialCombox(self):\n\
        def test(s,f):\n\
            if f>0.5:\n\
                return s\n\
            elif f<-0.5:\n\
                return '-'+s\n\
            return None\n\
        def getStr(vc):\n\
            for s,i in[('x',0),('y',1),('z',2)]:\n\
                ts = test(s,vc[i])\n\
                if ts:\n\
                    return ts\n\
        self.isAxialEnable = False\n\
        mat = cmds.getAttr('{nodeName}.influenceInfo[{index}].globalOffsetMat'.format(nodeName=self.nodeName,index=self.indexList[0]))\n\
        front = getStr(mat[0:3])\n\
        up = getStr(mat[4:7])\n\
        self.frontCombox.setCurrentText(front)\n\
        self.isAxialEnable = True\n\
        self.on_frontComb(front)\n\
        self.isAxialEnable = False\n\
        self.upCombox.setCurrentText(up)\n\
        self.isAxialEnable = True\n\
    def on_alignParentButton(self):\n\
        for nodeName,null,indexList in self.nodeInfList:\n\
            for index in indexList:\n\
                cmds.collisionOffsetSkin(nodeName,alignParent=index)\n\
        cmds.collisionOffsetSkin(refreshManipulators=True)\n\
    def on_alignChildButton(self):\n\
        for nodeName,null,indexList in self.nodeInfList:\n\
            for index in indexList:\n\
                cmds.collisionOffsetSkin(nodeName,alignChild=index)\n\
        cmds.collisionOffsetSkin(refreshManipulators=True)\n\
    def on_meshComb(self,index):\n\
        skinParameterWidget.comBoxText = self.meshComb.currentText()\n\
        self.nodeName = self.nodeInfList[index][0]\n\
        self.indexList = self.nodeInfList[index][2]\n\
        self.jointLengthP.refreshValue()\n\
        self.jointLengthC.refreshValue()\n\
        self.twistCutWeightP.refreshValue()\n\
        self.twistCutWeightC.refreshValue()\n\
        self.outsideCutProportion.refreshValue()\n\
        self.collisionHeight.refreshValue()\n\
        self.collisionDeep.refreshValue()\n\
        self.collisionOffset.refreshValue()\n\
        self.outsideHeight.refreshValue()\n\
        self.startAngleCos.refreshValue()\n\
        self.endAngleCos.refreshValue()\n\
        self.collisionSmoothP.refreshValue()\n\
        self.collisionSmoothC.refreshValue()\n\
        self.sideSmoothP.refreshValue()\n\
        self.sideSmoothC.refreshValue()\n\
        self.outsideExtendWeightP.refreshValue()\n\
        self.outsideExtendWeightC.refreshValue()\n\
        self.refreshIsLinearCheck()\n\
        self.refreshAxialCombox()\n\
    def on_frontComb(self,text):\n\
        if self.isAxialEnable:\n\
            axials = ['x','y','z','-x','-y','-z']\n\
            for axialPair in [['x','-x'],['y','-y'],['z','-z']]:\n\
                if text in axialPair:\n\
                    axials.remove(axialPair[0])\n\
                    axials.remove(axialPair[1])\n\
                    break\n\
            self.upCombox.clear()\n\
            self.upCombox.addItems(axials)\n\
    def on_upComb(self,upText):\n\
        if self.isAxialEnable:\n\
            frontText = self.frontCombox.currentText()\n\
            if frontText and upText:\n\
                zVector = self.vectorDic[frontText]^self.vectorDic[upText]\n\
                mat = [self.vectorDic[frontText][0],self.vectorDic[frontText][1],self.vectorDic[frontText][2],0,\n\
                       self.vectorDic[upText][0],self.vectorDic[upText][1],self.vectorDic[upText][2],0,\n\
                       zVector[0],zVector[1],zVector[2],0,\n\
                       0,0,0,1]\n\
                for nodeName,null,indexList in self.nodeInfList:\n\
                    for index in indexList:\n\
                        cmds.setAttr('{nodeName}.influenceInfo[{index}].globalOffsetMat'.format(\n\
                            nodeName = nodeName,index=index),mat,type='matrix')\n\
                cmds.collisionOffsetSkin(refreshManipulators=True)\n\
    def mousePressEvent(self,e):\n\
        if e.button() == QtCore.Qt.LeftButton:\n\
            self.point = e.globalPos() - self.pos()\n\
            self.isDrag = True\n\
            e.accept()\n\
    def mouseMoveEvent(self,e):\n\
        if self.isDrag:\n\
            skinParameterWidget.windowPosition = e.globalPos() - self.point\n\
            self.move(skinParameterWidget.windowPosition)\n\
            e.accept()\n\
    def mouseReleaseEvent(self,e):\n\
        self.isDrag = False\n\
    def paintEvent(self,e):\n\
        painter = QtGui.QPainter(self)\n\
        painter.setBrush(QtGui.QColor(50,50,50,255))\n\
        painter.setPen(QtGui.QColor(50,50,50,255))\n\
        painter.drawRoundedRect(self.rect(),20,20)\n\
    @staticmethod\n\
    def showWindow(nodeInfList):\n\
        if skinParameterWidget.window:\n\
            skinParameterWidget.window.deleteLater()\n\
        skinParameterWidget.window = skinParameterWidget(nodeInfList)\n\
        skinParameterWidget.window.show()\n\
    @staticmethod\n\
    def deleteWindow():\n\
        if skinParameterWidget.window:\n\
            skinParameterWidget.window.deleteLater()\n\
        skinParameterWidget.window=None\n\
";

const char* pythonScript3 = "class CreateSmoothSkinWin(QtWidgets.QWidget):\n\
    value = 0\n\
    def __init__(self,create=True,parent=None):\n\
        self.create = create\n\
        self.value = 0\n\
        self.slideMax=20\n\
        mainWindow = OpenMayaUI.MQtUtil.mainWindow()\n\
        qtmainwindow = wrapInstance(long(mainWindow),QtWidgets.QWidget)\n\
        super(CreateSmoothSkinWin,self).__init__(qtmainwindow)\n\
        self.setWindowFlags(QtCore.Qt.Window)\n\
        if create:\n\
            self.setWindowTitle(u'创建蒙皮')\n\
        else:\n\
            self.setWindowTitle(u'平滑蒙皮权重')\n\
        mainLayout = QtWidgets.QVBoxLayout()\n\
        self.setLayout(mainLayout)\n\
        layout1 = QtWidgets.QHBoxLayout()\n\
        layout1.setMargin(7)\n\
        mainLayout.addLayout(layout1)\n\
        label = QtWidgets.QLabel(u'weight smoothing times')\n\
        label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter)\n\
        self.numberLine = QtWidgets.QLineEdit()\n\
        self.numberLine.setFixedWidth(70)\n\
        self.numberLine.setValidator(QtGui.QIntValidator(0,999999))\n\
        self.numberLine.setText('0')\n\
        layout1.addWidget(label)\n\
        layout1.addWidget(self.numberLine)\n\
        self.numberLine.editingFinished.connect(self.on_editingFinished)\n\
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)\n\
        self.slider.setMaximum(self.slideMax)\n\
        self.slider.setMinimum(0)\n\
        layout1.addWidget(self.slider)\n\
        self.slider.sliderMoved.connect(self.on_sliderMoved)\n\
        if create:\n\
            createButton = QtWidgets.QPushButton(u'create skin')\n\
        else:\n\
            createButton = QtWidgets.QPushButton(u'smooth weiths')\n\
        createButton.clicked.connect(self.on_createButton)\n\
        mainLayout.addWidget(createButton)\n\
    def on_editingFinished(self):\n\
        try:\n\
            self.value = int(self.numberLine.text())\n\
            if self.value>self.slideMax:\n\
                self.slideMax = self.value\n\
                self.slider.setMaximum(self.slideMax)\n\
            self.slider.setValue(self.value)\n\
        except:\n\
            pass\n\
    def on_sliderMoved(self,p):\n\
        self.value = p\n\
        self.numberLine.setText('%i'%self.value)\n\
    def sizeHint(self):\n\
        return QtCore.QSize(300,150)\n\
    def on_createButton(self):\n\
        if self.create:\n\
            vpSkinScript.createVpSkin(self.value)\n\
        else:\n\
            vpSkinScript.smoothWeight(self.value)\n\
class helpWeidget(QtWidgets.QWidget):\n\
    w = None\n\
    def __init__(self):\n\
        mainwindow = OpenMayaUI.MQtUtil.mainWindow()\n\
        qtmainwindow = wrapInstance(long(mainwindow),QtWidgets.QWidget)\n\
        super(helpWeidget,self).__init__(qtmainwindow)\n\
        mainLayout = QtWidgets.QVBoxLayout()\n\
        self.setLayout(mainLayout)\n\
        label1 = QtWidgets.QLabel(u'效果视频<a href=\"https://vimeo.com/319290748\">vimeo')\n\
        label2 = QtWidgets.QLabel(u'制作步骤视频<a href=\"https://vimeo.com/319291596\">vimeo')\n\
        label3 = QtWidgets.QLabel(u'最新版关注<a href=\"https://github.com/t1280013890/collisionOffsetSkin/tree/master\">github')\n\
        mainLayout.addWidget(label1)\n\
        mainLayout.addWidget(label2)\n\
        mainLayout.addWidget(label3)\n\
        self.setWindowFlags(QtCore.Qt.Window)\n\
        label1.linkActivated.connect(self.openUrl)\n\
        label2.linkActivated.connect(self.openUrl)\n\
        label3.linkActivated.connect(self.openUrl)\n\
    def openUrl(self,url):\n\
        QtGui.QDesktopServices.openUrl(QtCore.QUrl(url))\n\
    @staticmethod\n\
    def showHelp():\n\
        helpWeidget.w = helpWeidget()\n\
        helpWeidget.w.show()\n\
class vpSkinScript:\n\
    vpSkinWindow = None\n\
    skinContext = None\n\
    @staticmethod\n\
    def createVpSkin(smooth):\n\
        def dis(a,b):\n\
            return pow(pow(a[0]-b[0],2)+pow(a[1]-b[1],2)+pow(a[2]-b[2],2),0.5)\n\
        jointList = cmds.ls(sl=True,type='joint')\n\
        if not jointList:\n\
            return\n\
        jointPairList = []\n\
        for joint in jointList:\n\
            parent = cmds.listRelatives(joint,p=True,type='joint',pa=True)\n\
            if not parent:\n\
                parent = [joint]\n\
            jointPoint = cmds.xform(joint,q=True,ws=True,t=True)\n\
            parentPoint = cmds.xform(parent[0],q=True,ws=True,t=True)\n\
            childJoints = cmds.listRelatives(joint,c=True,type = 'joint',pa=True)\n\
            parentLength = dis(jointPoint,parentPoint)\n\
            if parentLength<0.001:\n\
                parentLength = 1\n\
            jointLength = 1\n\
            if childJoints:\n\
                jointLength = 0\n\
                for child in childJoints:\n\
                    childPoint = cmds.xform(child,q=True,ws=True,t=True)\n\
                    jointLength += dis(jointPoint,childPoint)\n\
                jointLength /= len(childJoints)\n\
            else:\n\
                jointLength = parentLength\n\
            jointPairList.append([joint,parent[0],jointLength,parentLength])\n\
        meshTrans = [trans for trans in cmds.ls(sl=True,type='transform') if cmds.listRelatives(trans,shapes=True,type='mesh')]\n\
        for meshItem in meshTrans:\n\
            skinNode = cmds.deformer(meshItem,type='collisionOffsetSkin')[0]\n\
            for i,(joint,parentJoint,jointLength,parentLength) in enumerate(jointPairList):\n\
                cmds.connectAttr(joint+'.worldMatrix[0]',skinNode+'.matrix[%i]'%i)\n\
                cmds.connectAttr(parentJoint+'.worldMatrix[0]',skinNode+'.bindPreMatrix[%i]'%i)\n\
                jointMat =cmds.getAttr(joint+'.wm')\n\
                parentJointMat = cmds.getAttr(parentJoint+'.wm')\n\
                cmds.setAttr(skinNode + '.influenceInfo[%i].bindMat' % i, jointMat, type, type='matrix')\n\
                cmds.setAttr(skinNode + '.influenceInfo[%i].bindParentMat' % i, parentJointMat, type, type='matrix')\n\
                cmds.setAttr(skinNode + '.influenceInfo[%i].jointLengthC' % i, jointLength, type)\n\
                cmds.setAttr(skinNode + '.influenceInfo[%i].jointLengthP' % i, parentLength, type)\n\
                if not cmds.objExists(joint+'.lockInfluenceWeights'):\n\
                    cmds.addAttr(joint,sn='liw',ln='lockInfluenceWeights',at='bool')\n\
                cmds.connectAttr(joint+'.liw',skinNode+'.lockWeights[%i]'%i)\n\
            cmds.collisionOffsetSkin(skinNode,reBind=True)\n\
            cmds.skinCluster(skinNode,e=True,maximumInfluences=len(jointPairList))\n\
            cmds.collisionOffsetSkin(skinNode,initWeight=1,smoothWeight=smooth)\n\
            cmds.getAttr(skinNode+'.paintWeights')\n\
    @staticmethod\n\
    def smoothWeight(smooth):\n\
        objs = cmds.ls(sl=True,type='transform')\n\
        nameList = []\n\
        for obj in objs:\n\
            for shape in cmds.listRelatives(obj):\n\
                if not cmds.getAttr(shape+'.io'):\n\
                    nameList.append(shape)\n\
        skinDict = {}\n\
        for name in nameList:\n\
            try:\n\
                slls = OpenMaya.MSelectionList()\n\
                OpenMaya.MGlobal.getSelectionListByName(name,slls)\n\
            except:\n\
                continue\n\
            nodeObj = OpenMaya.MObject()\n\
            nodeDag = OpenMaya.MDagPath()\n\
            slls.getDependNode(0,nodeObj)\n\
            slls.getDagPath(0,nodeDag)\n\
            mitGeo = OpenMaya.MItGeometry(nodeObj)\n\
            compomentSL = OpenMaya.MSelectionList()\n\
            while(not mitGeo.isDone()):\n\
                compomentSL.add(nodeDag,mitGeo.currentItem(),True)\n\
                mitGeo.next()\n\
            compoment = OpenMaya.MObject()\n\
            compomentSL.getDagPath(0,nodeDag,compoment)\n\
            mitDG = OpenMaya.MItDependencyGraph(nodeObj,OpenMaya.MFn.kSkinClusterFilter,OpenMaya.MItDependencyGraph.kUpstream)\n\
            if mitDG.isDone():\n\
                continue\n\
            skinFn = OMA.MFnSkinCluster(mitDG.currentItem())\n\
            skinNode = skinFn.name()\n\
            cmds.collisionOffsetSkin(skinNode,smoothWeight=smooth)\n\
    @staticmethod\n\
    def initialize():\n\
        vpSkinScript.skinContext = cmds.skinManipContext()\n\
        vpSkinScript.vpSkinWindow = cmds.menu(u'COskin',p='MayaWindow',tearOff=True)\n\
        cmds.menuItem(l=u'create skin',c='CreateSmoothSkinWin().show()',stp = 'python',p = vpSkinScript.vpSkinWindow)\n\
        cmds.menuItem(l=u'smooth weights', c='CreateSmoothSkinWin(False).show()', stp='python', p=vpSkinScript.vpSkinWindow)\n\
        cmds.menuItem(l=u'parameter edit tool',c='cmds.setToolTo(vpSkinScript.skinContext)',stp='python', p=vpSkinScript.vpSkinWindow)\n\
        cmds.menuItem(l=u'帮助', c='helpWeidget.showHelp()', stp='python',p=vpSkinScript.vpSkinWindow)\n\
    @staticmethod\n\
    def uninitialize():\n\
        cmds.deleteUI(vpSkinScript.vpSkinWindow)\n\
";

const char* initializeCmd = "vpSkinScript.initialize()";
const char* uninitializeCmd = "vpSkinScript.uninitialize()";
const char* AETemplateCmd = "\
global proc AEcollisionOffsetSkinTemplate(string $nodeName) {\
	editorTemplate - suppress \"influenceInfo\";\
	AEskinClusterTemplate($nodeName);\
}";