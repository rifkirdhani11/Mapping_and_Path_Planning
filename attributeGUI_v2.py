import os
import sys
sys.path.append('..')

from tkinter import *
from tkinter.ttk import Progressbar
from tkinter import ttk, filedialog, messagebox
from ttkthemes import ThemedStyle 
from PIL import ImageTk,Image 

def tab(window, loadmap1, pathMaker, pathMakerStop, zeroPositionSet, viewdata1, senddata1, connect1, simulate1, disconnect,  
        loadmap2, boundaryMaker, boundaryMakerStop, polygon, viewdata2, senddata2, connect2, simulate2,
        pathFlagBtn, boundaryFlagBtn, mapTypeBtn):

    tab_control = ttk.Notebook(window)
    tab1 = ttk.Frame(tab_control)
    tab2 = ttk.Frame(tab_control)
    tab_control.add(tab1, text='Mapping')
    tab_control.add(tab2, text='Path Planning')
    tab_control.pack(expand=5, fill='both')

    tab_control2 = ttk.Notebook(tab2)
    tab2_1 = ttk.Frame(tab_control2)
    tab2_2 = ttk.Frame(tab_control2)
    tab_control2.add(tab2_1, text="Manual")
    tab_control2.add(tab2_2, text="Auto")
    tab_control2.pack(expand=1, fill='both')

    #style = ThemedStyle(tab2_1)
    #style.set_theme("arc")

    #tab2_1_frame1
    filemap1F = LabelFrame(tab2_1, text="File Map")
    filemap1F.grid(row=0, column=0, columnspan=3, sticky=W, padx=3, pady=3)
    lbl3 = Label(filemap1F, text="Scale")
    lbl3.grid(row=0, column=0, sticky=W, padx=3)
    scale = Entry(filemap1F, width=5)
    scale.grid(row=0, column=1, sticky=W, padx=3)
    scale.insert(END,'1')
    browse = Button(filemap1F, text="Add Image", command=loadmap1)
    browse.grid(row=0, column=2, sticky=W, padx=3)
    oriMap1 = Radiobutton(filemap1F, text='Original Map', value=0, variable=mapTypeBtn)
    oriMap1.grid(row=1, column=0, columnspan=3, sticky=W, padx=3)
    edgeMap1 = Radiobutton(filemap1F, text='Edge Map', value=1, variable=mapTypeBtn)
    edgeMap1.grid(row=2, column=0, columnspan=3, sticky=W, padx=3)
     
    #tab2_1_frame2
    path1F = LabelFrame(tab2_1, text="Path Planner")
    path1F.grid(row=0, column=3, columnspan=2, sticky=W, padx=3, pady=3)
    pointImg = Radiobutton(path1F, text='Enable Path', value=1, command=pathMaker, variable=pathFlagBtn)
    pointImg.grid(row=0, column=0, sticky=W, padx=3)
    unPointImg = Radiobutton(path1F, text='Disable Path', value=0, command=pathMakerStop, variable=pathFlagBtn)
    unPointImg.grid(row=1, column=0, sticky=W, padx=3)
    setZero1 = Radiobutton(path1F, text='Set Zero Point', value=2, command=zeroPositionSet, variable=pathFlagBtn)
    setZero1.grid(row=2, column=0, sticky=W, padx=3)

    #tab2_1_frame3
    pathData1F = LabelFrame(tab2_1, text="Path Data")
    pathData1F.grid(row=0, column=6, columnspan=2, sticky=W, padx=3, pady=3)
    viewData1 = Button(pathData1F, text="View", command=viewdata1)
    viewData1.grid(row=0, column=0, sticky=W, padx=3)
    sendData1 = Button(pathData1F, text="Send", command=senddata1)
    sendData1.grid(row=0, column=1, sticky=W, padx=3)
    sim1 = Button(pathData1F, text="Simualte", command=simulate1)
    sim1.grid(row=1, column=0, columnspan=2, sticky=W, padx=3)

    #tab2_1_frame4
    helpF1 = LabelFrame(tab2_1, text="Help")
    helpF1.grid(row=0, column=8, sticky=W, padx=3, pady=3)
    help1 = Button(helpF1, text="Help?", command=instruction1)
    help1.pack(side=LEFT, padx=3, pady=3)

    #tab2_1_frame5
    com1F = LabelFrame(tab2_1, text="Serial UART")
    com1F.grid(row=0, column=9, columnspan=4, sticky=W, padx=3, pady=3)
    lbl4 = Label(com1F, text="Baud")
    lbl4.grid(row=0, column=0, sticky=W, padx=3, pady=1)
    baud1 = Entry(com1F, width=7)
    baud1.grid(row=0, column=1, sticky=W, padx=3, pady=1)
    lbl5 = Label(com1F, text="Port")
    lbl5.grid(row=1, column=0, sticky=W, padx=3, pady=1)
    port1 = Entry(com1F, width=7)
    port1.grid(row=1, column=1, sticky=W, padx=3, pady=1)
    connect1 = Button(com1F, text="Connect", command=connect1)
    connect1.grid(row=0, column=2, sticky=W, padx=3, pady=1)
    disconnect1 = Button(com1F, text="Disonnect", command=disconnect)
    disconnect1.grid(row=1, column=2, sticky=W, padx=3, pady=1)

    #tab2_2_frame1
    filemap2F = LabelFrame(tab2_2, text="File Map")
    filemap2F.grid(row=0, column=0, columnspan=3, sticky=W, padx=3, pady=3)
    lbl6 = Label(filemap2F, text="Scale")
    lbl6.grid(row=0, column=0, sticky=W, padx=3)
    scale2 = Entry(filemap2F, width=5)
    scale2.grid(row=0, column=1, sticky=W, padx=3)
    scale2.insert(END,'1')
    loadImg2 = Button(filemap2F, text="Add Image", command=loadmap2)
    loadImg2.grid(row=0, column=2, sticky=W, padx=3)
    oriMap2 = Radiobutton(filemap2F, text='Original Map', value=0, variable=mapTypeBtn)
    oriMap2.grid(row=1, column=0, columnspan=3, sticky=W, padx=3)
    edgeMap2 = Radiobutton(filemap2F, text='Edge Map', value=1, variable=mapTypeBtn)
    edgeMap2.grid(row=2, column=0, columnspan=3, sticky=W, padx=3)

    #tab2_2_frame2
    path2F = LabelFrame(tab2_2, text="Path Planner")
    path2F.grid(row=0, column=3, rowspan=3, columnspan=2, sticky=W, padx=3, pady=3)
    BoundPoint = Radiobutton(path2F, text='Enable Corner', value=1, command=boundaryMaker, variable=boundaryFlagBtn)
    BoundPoint.grid(row=0, column=0, sticky=W, padx=3)
    unBoundPoint = Radiobutton(path2F, text='Disable Corner', value=0, command=boundaryMakerStop, variable=boundaryFlagBtn)
    unBoundPoint.grid(row=1, column=0, sticky=W, padx=3)
    setZero2 = Radiobutton(path2F, text='Set Zero Point', value=2, command=zeroPositionSet, variable=boundaryFlagBtn)
    setZero2.grid(row=2, column=0, sticky=W, padx=3)
    rasioIn = Entry(path2F, width=3)
    rasioIn.grid(row=0, column=1, sticky=W, padx=3, pady=1)
    rasioIn.insert(END,'1')
    lbl5 = Label(path2F, text="Ratio")
    lbl5.grid(row=0, column=2, sticky=W, padx=1, pady=1)
    genPath = Button(path2F, text="Generate", command=polygon)
    genPath.grid(row=1, column=1, columnspan=2, padx=3, pady=2, sticky=W)

    #tab2_2_frame3
    pathData2F = LabelFrame(tab2_2, text="Path Data")
    pathData2F.grid(row=0, column=6, columnspan=2, sticky=W, padx=3, pady=3)
    viewData2 = Button(pathData2F, text="View", command=viewdata2)
    viewData2.grid(row=0, column=0, sticky=W, padx=3)
    sendData2 = Button(pathData2F, text="Send", command=senddata2)
    sendData2.grid(row=0, column=1, sticky=W, padx=3)
    sim1 = Button(pathData2F, text="Simualte", command=simulate2)
    sim1.grid(row=1, column=0, columnspan=2, sticky=W, padx=3)

    #tab2_2_frame4
    helpF2 = LabelFrame(tab2_2, text="Help")
    helpF2.grid(row=0, column=8, sticky=W, padx=3, pady=3)
    help2 = Button(helpF2, text="Help?", command=instruction2)
    help2.pack(side=LEFT, padx=3, pady=3)

    #tab2_2_frame5
    com2F = LabelFrame(tab2_2, text="Serial UART")
    com2F.grid(row=0, column=9, columnspan=4, sticky=W, padx=3, pady=3)
    lbl7 = Label(com2F, text="Baud")
    lbl7.grid(row=0, column=0, sticky=W, padx=3, pady=1)
    baud2 = Entry(com2F, width=7)
    baud2.grid(row=0, column=1, sticky=W, padx=3, pady=1)
    lbl8 = Label(com2F, text="Port")
    lbl8.grid(row=1, column=0, sticky=W, padx=3, pady=1)
    port2 = Entry(com2F, width=7)
    port2.grid(row=1, column=1, sticky=W, padx=3, pady=1)
    connect2 = Button(com2F, text="Connect", command=connect2)
    connect2.grid(row=0, column=2, sticky=W, padx=3, pady=1)
    disconnect2 = Button(com2F, text="Disonnect", command=disconnect)
    disconnect2.grid(row=1, column=2, sticky=W, padx=3, pady=1)
    
    #bar = Progressbar(tab1, orient=HORIZONTAL, length=250)#, style='black.Horizontal.TProgessbar')
    #bar['value'] = 10
    #bar.place(x=10, y=70)

    return tab1, tab2_1, tab2_2, scale, scale2, rasioIn, port1, baud1, port2, baud2

def canvas1(tab2_1, filemap, imShow, callback):
    #view3 = Toplevel()
    #view3.title("Orthoimage Map")
    #view3.geometry('500x500')

    canvasImg = Canvas(tab2_1, width=imShow.width(), height=imShow.height())
    canvasImg.place(x=10, y=110)#grid(row=7, column=0, columnspan=7, rowspan=2, pady=10, sticky=W)#pack(pady=80)#pady55
    canvasImg.create_image(0,0, anchor=NW, image=imShow)
    
    scrolly = Scrollbar(canvasImg, orient=VERTICAL, command=canvasImg.yview)
    scrolly.place(relx=1, rely=0, relheight=1, anchor=NE)
    scrolly.bind("<MouseWheel>")
    scrollx = Scrollbar(canvasImg, orient=HORIZONTAL, command=canvasImg.xview)
    scrollx.place(relx=0, rely=1, relwidth=1, anchor=SW)

    canvasImg.bind("<Button-1>", callback)   
    canvasImg.config(xscrollcommand=scrollx.set, 
                    yscrollcommand=scrolly.set, 
                    #scrollregion=(0,0,imShow.width(),imShow.height()))
                    scrollregion=canvasImg.bbox(ALL))

def canvas2(tab2_2, filemap, imShow, callback):
    #scrolly = Scrollbar(tab2_2, orient=VERTICAL)
    #scrolly.place(relx=1, rely=0, relheight=1, anchor=NE)
    
    canvasImg = Canvas(tab2_2, width=imShow.width(), height=imShow.height())
    canvasImg.place(x=10, y=110) #grid(row=7, column=0, columnspan=7, rowspan=2, pady=10, sticky=W)#pack(expand=1)#pady55
    canvasImg.create_image(0,0, anchor=NW, image=imShow)
    
    #scrolly.config(command=canvasImg.yview)
    
    scrolly = Scrollbar(tab2_2, orient=VERTICAL, command=canvasImg.yview)
    scrolly.place(relx=1, rely=0, relheight=1, anchor=NE)
    scrolly.bind("<MouseWheel>")
    scrollx = Scrollbar(tab2_2, orient=HORIZONTAL, command=canvasImg.xview)
    scrollx.place(relx=0, rely=1, relwidth=1, anchor=SW)
    
    canvasImg.bind("<Button-1>", callback)
    #canvasImg.bind("<Button-3>", polygon)     
    canvasImg.config(xscrollcommand=scrollx.set, 
                    yscrollcommand=scrolly.set, 
                    #scrollregion=(0,0,imShow.width(),imShow.height()))
                    scrollregion=canvasImg.bbox(ALL))
                    
    return canvasImg

def instruction1():
    messagebox.showinfo('HELP', '1. Set the scalling ratio of image. Fill the value on Scale of Image.\n'+
                                '2. Load the orthoimage (map) to the workspace.\n'+
                                '3. Select zero point coordinate. Click ZERO button.\n'+
                                '4. Build your own path. Click START to begin build path.\n'+
                                '5. Click the the coordinates on the workspace. Click STOP to end the process.\n')

def instruction2():
    messagebox.showinfo('HELP', '1. Load the orthoimage (map) to the workspace.\n'+
                                '2. Select zero point coordinate. Click ZERO button.\n'+
                                '3. Select boundary of area. Click the point to make polygon who cover the area.\n'+
                                '4. Set the ratio of path. Ratio mean the diameter and the safe distance of Mobile Robot.\n'+
                                '5. Build path. Start coverage path planning. Click GENERATE button.\n')

