import os
import sys
sys.path.append('..')
import math
import numpy as np
import CPplanner, ODMapper, attributeGUI_v2, hed

from serial import *
from tkinter import *
from tkinter.ttk import Progressbar
from tkinter import ttk, filedialog, scrolledtext
from PIL import ImageTk,Image 

root = Tk()
root.iconbitmap('pens.ico')
root.title("ROBOTANI Mapping and Path Planning")
root.geometry('700x350')
root.config(cursor='arrow red red')

s = ttk.Style()
s.configure('TNotebook.Tab', font=('Segoe UI','11'))

frstPthFlagMan, frstPthFlagAuto = 0, 0
x1, x2, xZ, y1, y2, yZ, ox, oy = 0, 0, 0, 0, 0, 0, 0, 0
pjg, pjgTot, l, j, finalPath = 0, 0, 0, 0, 0
pathFlagBtn=IntVar()
boundaryFlagBtn=IntVar()
mapTypeBtn=IntVar()
#serial_data=''
serial_object=None

def pathMaker():
    root.config(cursor='plus red red')
def pathMakerStop():
    root.config(cursor='arrow red red')
def zeroPositionSet():
    root.config(cursor='plus red red')
def boundaryMaker():
    root.config(cursor='plus red red')
def boundaryMakerStop():
    root.config(cursor='arrow red red')

def clickManu(event):
    global frstPthFlagMan, j, x1, x2, xZ, y1, y2, yZ, l, pjg, pjgTot, ox, oy, poly
    canvasImg = event.widget
    print ("clicked at ", event.x, event.y)
    pathFlag  = pathFlagBtn.get()

    if frstPthFlagMan==1 and pathFlag==1: #2nd and above click
        j = j+1
        x2 = canvasImg.canvasx(event.x)
        y2 = canvasImg.canvasy(event.y)
        
        poly.append(x2-xZ)
        poly.append(y2-yZ)
        
        canvasImg.create_oval(x2-5, y2-5, x2+5, y2+5, outline="#f11", fill="#fff", width=2)
        canvasImg.create_line(x1,y1,x2,y2, fill="#f22", width=3)

        f = open("coordinateManu.txt", "a+")
        f.write("id_%d, x_%d, y_%d, l_%d\r" % (j,(x2-xZ),(y2-yZ),pjgTot))
        f.close()
    
        l = math.sqrt((math.pow((x2-x1),2)+(math.pow((y2-y1),2))))
        pjg = math.ceil(l*100)/100
        pjgTot = pjgTot + pjg
        print(j)
        
        x1 = x2
        y1 = y2

    elif frstPthFlagMan==0 and pathFlag==1: #first click 
        x1 = canvasImg.canvasx(event.x)
        y1 = canvasImg.canvasy(event.y)

        poly = [(x1-xZ), (y1-yZ)]
        
        canvasImg.create_oval(x1-5, y1-5, x1+5, y1+5, outline="#f11", fill="#fff", width=2)      
        frstPthFlagMan = 1
    
    elif pathFlag==2: #Set zero point
        xZ = canvasImg.canvasx(event.x)
        yZ = canvasImg.canvasy(event.y)
        #point indicator
        canvasImg.create_rectangle(xZ-2, yZ-8, xZ+2, yZ+8, outline="#f11", fill="#1f1", width=2) 
        canvasImg.create_rectangle(xZ-8, yZ-2, xZ+8, yZ+2, outline="#f11", fill="#1f1", width=2) 
        #line edges
        canvasImg.create_line(xZ,0,xZ,imShow.height(), fill="#f22", width=1)
        canvasImg.create_line(0,yZ,imShow.width(),yZ, fill="#f22", width=1)

def clickAuto(event):
    global frstPthFlagAuto, x1, x2, xZ, y1, y2, yZ, ox, oy, poly, corner, corner2
    canvasImg = event.widget
    print ("clicked at ", event.x, event.y)
    pathFlag  = pathFlagBtn.get()
    boundaryFlag = boundaryFlagBtn.get()

    if frstPthFlagAuto==1 and boundaryFlag==1:
        x2 = canvasImg.canvasx(event.x)
        y2 = canvasImg.canvasy(event.y)
        ox.append(x2-xZ)
        #oy.append(((abs(imShow.height()-y2))-yZ))
        oy.append(yZ-y2)
        poly.append(x2)
        poly.append(y2)
        corner2 = np.array((x2, y2))
        corner = np.vstack((corner, corner2))
        
        canvasImg.create_oval(x2-5, y2-5, x2+5, y2+5, outline="#f11", fill="#fff", width=2)

        x1 = x2
        y1 = y2

    elif frstPthFlagAuto==0 and (pathFlag==1 or boundaryFlag==1):
        x1 = canvasImg.canvasx(event.x)
        y1 = canvasImg.canvasy(event.y)

        ox = [(x1-xZ)]
        #oy = [((abs(imShow.height()-y1))-yZ)]
        oy = [yZ-y1]
        poly = [x1, y1]
        corner = np.array((x1, y1))
        
        canvasImg.create_oval(x1-5, y1-5, x1+5, y1+5, outline="#f11", fill="#fff", width=2)      
        frstPthFlagAuto = 1

    elif boundaryFlag==2: #Set zero point
        xZ = canvasImg.canvasx(event.x)
        yZ = canvasImg.canvasy(event.y)
        print(xZ, yZ)
        #point indicator
        canvasImg.create_rectangle(xZ-2, yZ-8, xZ+2, yZ+8, outline="#f11", fill="#1f1", width=2) 
        canvasImg.create_rectangle(xZ-8, yZ-2, xZ+8, yZ+2, outline="#f11", fill="#1f1", width=2) 
        #line edges
        canvasImg.create_line(xZ,0,xZ,imShow.height(), fill="#f22", width=1)
        canvasImg.create_line(0,yZ,imShow.width(),yZ, fill="#f22", width=1)

def polygon():
    global finalPath
    canvasImg.create_polygon(poly, fill="#f00", width=3)
    print(corner)
    #f = open("coordinateAuto.txt", "a+")
    f = open("coordinateAutoSim.txt", "a+")
    reso = int(rasioIn.get()) 
    px, py, pl = CPplanner.planning_animation(ox, oy, reso) #start CPP
    print(ox)
    n = len(py)
    finalPath = [px[0],py[0]]
    #f.write("x_%d, y_%d\r" % (px[0],py[0]))
    f.write("{%d, %d}," % (px[0],py[0]))
    i=1
    for i in range(n): #loging data coordinates
        f.write("{%d, %d}," % (px[i],py[i]))
        finalPath.append(px[i])
        finalPath.append(py[i])
    
    #print(finalPath)
    f.close()

def open_map():
    global filemap, img, imShow
    mapFlag = mapTypeBtn.get()
    i=0
    scl = int(scale.get())
    imScale = scl
    filemap = filedialog.askopenfilename(initialdir = "Data_Pengujian")

    if mapFlag==1:
        img = hed.run(filemap)
    else:
        img = Image.open(filemap)
    
    img = img.resize((int(img.size[0]/imScale), int(img.size[1]/imScale)), Image.ANTIALIAS)
    imShow = ImageTk.PhotoImage(img)
 
    attributeGUI_v2.canvas1(tab2_1, filemap, imShow, clickManu)

def open_map2():
    global filemap, img, imShow, canvasImg
    mapFlag = mapTypeBtn.get()
    i=0
    scl = int(scale.get())
    imScale = scl
    filemap = filedialog.askopenfilename(initialdir = "Data_Pengujian")
    
    if mapFlag==1:
        img = hed.run(filemap)
    else:
        img = Image.open(filemap)

    img = img.resize((int(img.size[0]/imScale), int(img.size[1]/imScale)), Image.ANTIALIAS)
    imShow = ImageTk.PhotoImage(img)

    canvasImg = attributeGUI_v2.canvas2(tab2_2, filemap, imShow, clickAuto)

def viewdata1():
    view1 = Toplevel()
    view1.title("Path Coordinate Viewer")
    view1.geometry("300x300+500+200")
    path1 = scrolledtext.ScrolledText(view1, width=500, height=200)
    path1.insert(INSERT, str(poly))
    path1.pack()

def senddata1():
    tfData = str.encode(str(poly) + '\n')
    serial_object.write(tfData)

def simulate1():
    print("sim1 ok")
    reso = int(rasioIn.get()) 
    CPplanner.simulate(corner, reso)

def viewdata2():
    view2 = Toplevel()
    view2.title("Path Coordinate Viewer")
    view2.geometry("300x300+500+200")
    path2 = scrolledtext.ScrolledText(view2, width=500, height=200)
    path2.insert(INSERT, str(finalPath))
    path2.pack()

def senddata2():
    tfData2 = str.encode(str(finalPath) + '\n')
    serial_object.write(tfData2)

def simulate2():
    print("sim2 ok")
    print(corner)
    reso = int(rasioIn.get()) 
    CPplanner.simulate(corner)

def connect1():
    global serial_object
    port = port1.get()
    baud = baud1.get()
    serial_object = Serial('COM'+str(port), baud)
    print(port + " " + baud)

def connect2():
    global serial_object
    port = port2.get()
    baud = baud2.get()
    serial_object = Serial('COM'+str(port), baud)
    print(port + " " + baud)

def disconnect():
    serial_object.close()
    
tab1, tab2_1, tab2_2, scale, scale2, rasioIn, port1, baud1, port2, baud2 = attributeGUI_v2.tab(root, open_map, pathMaker, pathMakerStop, zeroPositionSet, 
                                                                                               viewdata1, senddata1, connect1, simulate1, disconnect,
                                                                                               open_map2, boundaryMaker, boundaryMakerStop, polygon,
                                                                                               viewdata2, senddata2, connect2, simulate2,
                                                                                               pathFlagBtn, boundaryFlagBtn, mapTypeBtn)
ODMapper.attributeODM(root, tab1)


root.mainloop()
