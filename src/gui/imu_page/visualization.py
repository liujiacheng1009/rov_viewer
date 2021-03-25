#from PySide2 import QtCore, QtGui,QtWidgets
from PyQt5 import QtCore 
from PyQt5 import QtWidgets 
from PyQt5 import QtGui 
from OpenGL import GL,GLU,GLUT
from PyQt5.QtOpenGL import *

class Visualization(QtWidgets.QOpenGLWidget):
    def __init__(self,parent=None):
        super(Visualization,self).__init__(parent)
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0

    def initializeGL(self):
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_NORMALIZE)
        GL.glClearColor(0,0,0,1)

    def resizeGL(self,w,h):
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GLU.gluPerspective(40, float(w) / float(h),0.5, 20.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glViewport(0,0,w,h)

    def paintGL(self):
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()

        GL.glTranslatef(0,0,-2.5)
        GL.glRotatef(self.xRot+90, 1,0,0)
        GL.glRotatef(self.yRot, 0,1,0)
        GL.glRotatef(self.zRot+180,0,0,1)
        GL.glEnable(GL.GL_DEPTH_TEST)
        self.drawObject()
        self.drawWiredQuader(0.2,1,0.5)
        GL.glDisable(GL.GL_DEPTH_TEST)
        self.drawCoodinateSystem()

    def drawObject(self):
        '''
        绘制方盒的面
        '''
        GL.glColor4f(0.5,0.5,0,1)
        # z
        GL.glBegin(GL.GL_QUADS) ## 平面
        GL.glVertex3f(-0.5, -0.25,  0.1)
        GL.glVertex3f( 0.5, -0.25,  0.1)
        GL.glVertex3f( 0.5,  0.25,  0.1)
        GL.glVertex3f(-0.5,  0.25,  0.1)
        GL.glEnd()

        # -z
        GL.glBegin(GL.GL_QUADS)
        GL.glVertex3f( 0.5, -0.25, -0.1)
        GL.glVertex3f(-0.5, -0.25, -0.1)
        GL.glVertex3f(-0.5,  0.25, -0.1)
        GL.glVertex3f( 0.5,  0.25, -0.1)
        GL.glEnd()

        # x 
        GL.glBegin(GL.GL_QUADS)
        GL.glVertex3f( 0.5, -0.25,  0.1)
        GL.glVertex3f( 0.5, -0.25, -0.1)
        GL.glVertex3f( 0.5,  0.25, -0.1)
        GL.glVertex3f( 0.5,  0.25,  0.1)
        GL.glEnd()

        # -x 
        GL.glBegin(GL.GL_QUADS)
        GL.glVertex3f(-0.5, -0.25, -0.1)
        GL.glVertex3f(-0.5, -0.25,  0.1)
        GL.glVertex3f(-0.5,  0.25,  0.1)
        GL.glVertex3f(-0.5,  0.25, -0.1)
        GL.glEnd()

        # y 
        GL.glBegin(GL.GL_QUADS)
        GL.glVertex3f(-0.5,  0.25,  0.1)
        GL.glVertex3f( 0.5,  0.25,  0.1)
        GL.glVertex3f( 0.5,  0.25, -0.1)
        GL.glVertex3f(-0.5,  0.25, -0.1)
        GL.glEnd()

        # -y 
        GL.glBegin(GL.GL_QUADS)
        GL.glVertex3f(-0.5, -0.25, -0.1)
        GL.glVertex3f( 0.5, -0.25, -0.1)
        GL.glVertex3f( 0.5, -0.25,  0.1)
        GL.glVertex3f(-0.5, -0.25,  0.1)
        GL.glEnd()

    def drawWiredQuader(self,h,l,w):
        '''
        绘制方盒的边
        '''
        h /= 2
        l /= 2
        w /= 2
        GL.glDisable(GL.GL_DEPTH_TEST)
        GL.glColor4f(0.0,1.0,0.0,0.2)
        GL.glBegin(GL.GL_LINES)
        GL.glVertex3f(-l, -w,  h)
        GL.glVertex3f( l, -w,  h)

        GL.glVertex3f( l,  w,  h)
        GL.glVertex3f(-l,  w,  h)

        GL.glVertex3f(-l, -w, -h)
        GL.glVertex3f( l, -w, -h)

        GL.glVertex3f( l,  w, -h)
        GL.glVertex3f(-l,  w, -h)

        GL.glVertex3f(-l, -w,  h)
        GL.glVertex3f(-l, -w, -h)

        GL.glVertex3f( l, -w,  h)
        GL.glVertex3f( l, -w, -h)

        GL.glVertex3f(-l,  w,  h)
        GL.glVertex3f(-l,  w, -h)

        GL.glVertex3f( l,  w,  h)
        GL.glVertex3f( l,  w, -h)

        GL.glVertex3f(-l, -w,  h)
        GL.glVertex3f(-l,  w,  h)

        GL.glVertex3f(-l, -w, -h)
        GL.glVertex3f(-l,  w, -h)

        GL.glVertex3f( l, -w, -h)
        GL.glVertex3f( l,  w, -h)

        GL.glVertex3f( l,  w,  h)
        GL.glVertex3f( l, -w,  h)

        GL.glEnd()
        GL.glEnable(GL.GL_DEPTH_TEST)

    def drawCoodinateSystem(self):
        '''
        绘制坐标轴
        '''
        length = 0.8
        GL.glBegin(GL.GL_LINES)
        # x
        GL.glColor3f(1,0,0)
        GL.glVertex3f(0,0,0)
        GL.glVertex3f(length,0,0)
        # y
        GL.glColor3f(1,0,1)
        GL.glVertex3f(0,0,0)
        GL.glVertex3f(0,length,0)
        # z
        GL.glColor3f(0,0,1)
        GL.glVertex3f(0,0,0)
        GL.glVertex3f(0,0,length)
        GL.glEnd()

        # x-arrow
        GL.glPushMatrix()
        GL.glColor3f(1,0,0)
        GL.glTranslatef(length,0,0)
        GL.glRotatef(90,0,1,0)
        GLUT.glutSolidCone(length/38,length/18,10,10)
        GL.glTranslatef(-length,0,0)
        GL.glRotatef(-90,0,1,0)
        GL.glPopMatrix()

        # y-arrow
        GL.glPushMatrix()
        GL.glColor3f(1,0,1)
        GL.glTranslatef(0,length,0)
        GL.glRotatef(-90,1,0,0)
        GLUT.glutSolidCone(length/38,length/18,10,10)
        GL.glTranslatef(0,-length,0)
        GL.glRotatef(90,1,0,0)
        GL.glPopMatrix()

        # z-arrow
        GL.glPushMatrix()
        GL.glColor3f(0,0,1)
        GL.glTranslatef(0,0,length)
        GLUT.glutSolidCone(length/38,length/18,10,10)
        GL.glTranslatef(0,0,-length)
        GL.glPopMatrix()

    def updateAngles(self,r,p,y):
        self.xRot = int(p)
        self.yRot = int(r)
        self.zRot = -int(y)
        self.update()
