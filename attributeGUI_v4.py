import os
import sys
sys.path.append('..')

from tkinter import *
from tkinter.ttk import Progressbar
from tkinter import ttk, filedialog, messagebox
from ttkthemes import ThemedStyle 
from PIL import ImageTk,Image 

class Attrib:
	def __init__(self):
		self.wsize = 0
		self.hsize = 0
		self.reso = 10
		self.scale = 1

	def instruction1(self):
		messagebox.showinfo('HELP', '1. Set the scalling ratio of image. Fill the value on Scale of Image.\n'+
									'2. Load the orthoimage (map) to the workspace.\n'+
									'3. Select zero point coordinate. Click ZERO button.\n'+
									'4. Build your own path. Click START to begin build path.\n'+
									'5. Click the the coordinates on the workspace. Click STOP to end the process.\n')

	def instruction2(self):
		messagebox.showinfo('HELP', '1. Load the orthoimage (map) to the workspace.\n'+
									'2. Select zero point coordinate. Click ZERO button.\n'+
									'3. Select boundary of area. Click the point to make polygon who cover the area.\n'+
									'4. Set the ratio of path. Ratio mean the diameter and the safe distance of Mobile Robot.\n'+
									'5. Build path. Start coverage path planning. Click GENERATE button.\n')

	def tab(self, window, loadmap1, reset1, viewdata1, senddata1, simulate1, connect1, disconnect, boundaryMaker, boundaryMakerStop, zeroPositionSet,
			loadmap2, reset2, getBoundPoint, polygon, viewdata2, senddata2, simulate2, connect2, boundaryFlagBtn, zeroFlag, tMin, tMax):
		self.tab_control = ttk.Notebook(window)
		self.tab1 = ttk.Frame(self.tab_control)
		self.tab2 = ttk.Frame(self.tab_control)
		self.tab_control.add(self.tab1, text='Mapping')
		self.tab_control.add(self.tab2, text='Path Planning')
		self.tab_control.pack(expand=5, fill='both')

		self.tab_control2 = ttk.Notebook(self.tab2)
		self.tab2_1 = ttk.Frame(self.tab_control2)
		self.tab2_2 = ttk.Frame(self.tab_control2)
		self.tab_control2.add(self.tab2_1, text="Manual")
		self.tab_control2.add(self.tab2_2, text="Auto")
		self.tab_control2.pack(expand=1, fill='both')

		self.style = ThemedStyle(self.tab2_1)
		self.style.set_theme("vista")

		#tab2_1_frame1 File Map
		self.filemap1F = ttk.LabelFrame(self.tab2_1, text="File Map")
		self.filemap1F.grid(row=0, column=0, columnspan=3, sticky=NW, padx=3, pady=3)
		self.browse = ttk.Button(self.filemap1F, text="Add Image", command=loadmap1)
		self.browse.grid(row=0, column=0, sticky=W, padx=3)
		self.reset1 = ttk.Button(self.filemap1F, text="Reset Map", command=reset1)
		self.reset1.grid(row=1, column=0, sticky=W, padx=3)
		
		#tab2_1_frame2 Path Planner
		self.path1F = ttk.LabelFrame(self.tab2_1, text="Path Planner")
		self.path1F.grid(row=0, column=3, columnspan=2, sticky=NW, padx=3, pady=3)
		self.BoundPoint = ttk.Radiobutton(self.path1F, text='Enable Corner', value=1, command=boundaryMaker, variable=boundaryFlagBtn)
		self.BoundPoint.grid(row=0, column=0, sticky=W, padx=3)
		self.unBoundPoint = ttk.Radiobutton(self.path1F, text='Disable Corner', value=0, command=boundaryMakerStop, variable=boundaryFlagBtn)
		self.unBoundPoint.grid(row=1, column=0, sticky=W, padx=3)
		self.setZero1 = ttk.Checkbutton(self.path1F, text='Set Zero Point',  command=zeroPositionSet, variable=zeroFlag)
		self.setZero1.grid(row=2, column=0, sticky=W, padx=3)
		self.rasioIn1 = ttk.Entry(self.path1F, width=3)
		self.rasioIn1.grid(row=0, column=1, sticky=W, padx=3)
		self.rasioIn1.insert(END,'1')
		self.lblReso1 = ttk.Label(self.path1F, text="Reso")
		self.lblReso1.grid(row=0, column=2, sticky=W, padx=1)
		self.genPath = ttk.Button(self.path1F, text="Generate", command=polygon)
		self.genPath.grid(row=1, rowspan=2, column=1, columnspan=2, padx=3, sticky=W)

		#tab2_1_frame3 Path Data
		self.pathData1F = ttk.LabelFrame(self.tab2_1, text="Path Data")
		self.pathData1F.grid(row=0, column=6, columnspan=2, sticky=NW, padx=3, pady=3)
		self.viewData1 = ttk.Button(self.pathData1F, text="View", command=viewdata1)
		self.viewData1.grid(row=0, column=0, sticky=W, padx=3)
		self.sendData1 = ttk.Button(self.pathData1F, text="Send", command=senddata1)
		self.sendData1.grid(row=0, column=1, sticky=W, padx=3)
		self.sim1 = ttk.Button(self.pathData1F, text="Simualte", command=simulate1)
		self.sim1.grid(row=1, column=0, sticky=W, padx=3)
		self.help1 = ttk.Button(self.pathData1F, text="Help?", command=self.instruction1)
		self.help1.grid(row=1, column=1, sticky=W, padx=3)

		#tab2_1_frame5 Serial
		self.com1F = ttk.LabelFrame(self.tab2_1, text="Serial UART")
		self.com1F.grid(row=0, column=8, sticky=NW, padx=3, pady=3)
		self.lbl4 = ttk.Label(self.com1F, text="Baud")
		self.lbl4.grid(row=0, column=0, sticky=W, padx=3, pady=1)
		self.baud1 = ttk.Combobox(self.com1F, width=7)
		self.baud1['values']=(4800, 9600, 19200, 38400, 57600, 74880, 115200)
		self.baud1.current(4) #set the default item start form 0
		self.baud1.grid(row=0, column=1, columnspan=2, sticky=W, padx=3, pady=1)
		self.lbl5 = ttk.Label(self.com1F, text="Port")
		self.lbl5.grid(row=1, column=0, sticky=W, padx=3, pady=1)
		self.port1 = ttk.Entry(self.com1F, width=7)
		self.port1.grid(row=1, column=1, sticky=W, padx=1, pady=1)
		self.connectInd = Canvas(self.com1F, width=15, height=15, bg='red')
		self.connectInd.grid(row=1, column=2, sticky=W, padx=1, pady=1)
		self.connect1 = ttk.Button(self.com1F, text="Connect", command=connect1)
		self.connect1.grid(row=0, column=3, sticky=W, padx=3, pady=1)
		self.disconnect1 = ttk.Button(self.com1F, text="Disonnect", command=disconnect)
		self.disconnect1.grid(row=1, column=3, sticky=W, padx=3, pady=1)

		#self.tab2_2_frame1 File Map
		self.filemap2F = ttk.LabelFrame(self.tab2_2, text="File Map")
		self.filemap2F.grid(row=0, column=0, columnspan=3, sticky=NW, padx=3, pady=3)
		self.loadImg2 = ttk.Button(self.filemap2F, text="Add Image", command=loadmap2)
		self.loadImg2.grid(row=0, column=0, sticky=W, padx=3)
		self.reset2 = ttk.Button(self.filemap2F, text="Reset Map", command=reset2)
		self.reset2.grid(row=1, column=0, sticky=W, padx=3)

		#self.tab2_2_frame2 self.path Planner
		self.path2F = ttk.LabelFrame(self.tab2_2, text="Path Planner")
		self.path2F.grid(row=0, column=3, rowspan=3, columnspan=2, sticky=NW, padx=3, pady=3)
		self.treshMin = Spinbox(self.path2F, from_=0, to=255, textvariable=tMin, width=3)
		self.treshMin.grid(row=0, column=0, sticky=W, padx=3)
		self.lblThreshMin = ttk.Label(self.path2F, text="Threshold Min")
		self.lblThreshMin.grid(row=0, column=1, sticky=W, padx=1)
		#self.treshMin.insert(END,'160')
		self.treshMax = Spinbox(self.path2F, from_=0, to=255, textvariable=tMax, width=3)
		self.treshMax.grid( row=1, column=0, sticky=W, padx=3)
		self.lblThreshMax = ttk.Label(self.path2F, text="Threshold Max")
		self.lblThreshMax.grid(row=1, column=1, sticky=W, padx=1)
		self.getBound = ttk.Button(self.path2F, text="Get Boundary", command=getBoundPoint)
		self.getBound.grid(row=2, column=0, columnspan=2, padx=3, sticky=W)
		self.setZero2 = ttk.Checkbutton(self.path2F, text='Zero Point', command=zeroPositionSet, variable=boundaryFlagBtn)
		self.setZero2.grid(row=0, column=2, columnspan=2, sticky=W, padx=3)

		self.rasioIn2 = ttk.Entry(self.path2F, width=3)
		self.rasioIn2.grid(row=1, column=2, sticky=W, padx=3)
		self.rasioIn2.insert(END,'1')
		self.lblReso2 = ttk.Label(self.path2F, text="Reso")
		self.lblReso2.grid(row=1, column=3, sticky=W, padx=1)
		self.genPath = ttk.Button(self.path2F, text="Generate", command=polygon)
		self.genPath.grid(row=2, column=2, columnspan=2, padx=3, sticky=W)

		#self.tab2_2_frame3 Path Data
		self.pathData2F = ttk.LabelFrame(self.tab2_2, text="Path Data")
		self.pathData2F.grid(row=0, column=6, columnspan=2, sticky=NW, padx=3, pady=3)
		self.viewData2 = ttk.Button(self.pathData2F, text="View", command=viewdata2)
		self.viewData2.grid(row=0, column=0, sticky=W, padx=3)
		self.sendData2 = ttk.Button(self.pathData2F, text="Send", command=senddata2)
		self.sendData2.grid(row=0, column=1, sticky=W, padx=3)
		self.sim1 = ttk.Button(self.pathData2F, text="Simualte", command=simulate2)
		self.sim1.grid(row=1, column=0, sticky=W, padx=3)
		self.help2 = ttk.Button(self.pathData2F, text="Help?", command=self.instruction2)
		self.help2.grid(row=1, column=1,  sticky=W, padx=3)

		#tab2_2_frame5 Serial
		self.com2F = ttk.LabelFrame(self.tab2_2, text="Serial UART")
		self.com2F.grid(row=0, column=8, sticky=NW, padx=3, pady=3)
		self.lblBaud = ttk.Label(self.com2F, text="Baud")
		self.lblBaud.grid(row=0, column=0, sticky=W, padx=3, pady=1)
		self.baud2 = ttk.Combobox(self.com2F, width=7)
		self.baud2['values']=(4800, 9600, 19200, 38400, 57600, 74880, 115200)
		self.baud2.current(4) #set the default item start form 0
		self.baud2.grid(row=0, column=1, columnspan=2, sticky=W, padx=3, pady=1)
		self.lblPort = ttk.Label(self.com2F, text="Port")
		self.lblPort.grid(row=1, column=0, sticky=W, padx=3, pady=1)
		self.port2 = ttk.Entry(self.com2F, width=7)
		self.port2.grid(row=1, column=1, sticky=W, padx=1, pady=1)
		self.connectInd2 = Canvas(self.com2F, width=15, height=15, bg='red')
		self.connectInd2.grid(row=1, column=2, sticky=W, padx=1, pady=1)
		self.connect2 = ttk.Button(self.com2F, text="Connect", command=connect2)
		self.connect2.grid(row=0, column=3, sticky=W, padx=3, pady=1)
		self.disconnect2 = ttk.Button(self.com2F, text="Disonnect", command=disconnect)
		self.disconnect2.grid(row=1, column=3, sticky=W, padx=3, pady=1)

		return self.tab1, self.tab2_1, self.tab2_2, self.rasioIn1, self.rasioIn2, self.treshMin, self.treshMax, self.port1, self.baud1, self.port2, self.baud2, self.connectInd, self.connectInd2 #, self.im_in, self.im_ed

	def canvas1(self, tab2_1, imShow, callback):
		self.ox = []
		self.oy = []
		self.poly = []
		self.finalPath = []
		self.finalPathLine = []
		self.px = []
		self.py = []

		self.canvasImg1 = Canvas(self.tab2_1, width=imShow.width(), height=imShow.height())
		self.canvasImg1.place(x=10, y=110)#grid(row=7, column=0, columnspan=7, rowspan=2, pady=10, sticky=W)#pack(pady=80)#pady55
		self.canvasImg1.delete("all")
		self.canvasImg1.create_image(0,0, anchor=NW, image=imShow)

		self.canvasImg1.bind("<Button-1>", callback)
		
		return self.canvasImg1   

	def canvas2(self, tab2_2, imShowOri, imShow, callback):
		self.ox = []
		self.oy = []
		self.poly = []
		self.finalPath = []
		self.finalPathLine = []
		self.px = []
		self.py = []

		self.canvasImg2ori = Canvas(self.tab2_2, width=imShowOri.width(), height=imShowOri.height())
		self.canvasImg2ori.place(x=10, y=110) #grid(row=7, column=0, columnspan=7, rowspan=2, pady=10, sticky=W)#pack(expand=1)#pady55
		self.canvasImg2 = Canvas(self.tab2_2, width=imShow.width(), height=imShow.height())
		self.canvasImg2.place(x=510, y=110) #grid(row=7, column=0, columnspan=7, rowspan=2, pady=10, sticky=W)#pack(expand=1)#pady55
		self.canvasImg2.delete("all")
		self.putImageOri = self.canvasImg2ori.create_image(0,0, anchor=NW, image=imShowOri)
		self.putImage = self.canvasImg2.create_image(0,0, anchor=NW, image=imShow)
		self.canvasImg2ori.bind("<Button-1>", callback)
		self.canvasImg2.bind("<Button-1>", callback)

		return self.canvasImg2ori, self.canvasImg2, self.putImageOri, self.putImage

	def ind(self, data):
		#self.tab2_2_frame5 indikator
		self.ind1F = ttk.LabelFrame(self.tab2_1, text="Indicator")
		self.ind1F.grid(row=0, column=9, sticky=NW, padx=3, pady=3)
		self.size = ttk.Label(self.ind1F, text="Size ")
		self.size.grid(row=0, column=0, sticky=W, padx=1)
		self.scl = ttk.Label(self.ind1F, text="Scale ")
		self.scl.grid(row=1, column=0, sticky=W, padx=1)
		self.resol = ttk.Label(self.ind1F, text="Reso ")
		self.resol.grid(row=2, column=0, sticky=W, padx=1)
		self.area = ttk.Label(self.ind1F, text="Area ")
		self.area.grid(row=0, column=2, sticky=W, padx=1)
		self.lgpth = ttk.Label(self.ind1F, text="Long Path ")
		self.lgpth.grid(row=1, column=2, sticky=W, padx=1)
		self.trng = ttk.Label(self.ind1F, text="Boundary ")
		self.trng.grid(row=2, column=2, sticky=W, padx=1)
		self.bndr = ttk.Label(self.ind1F, text="Turning ")
		self.bndr.grid(row=0, column=4, sticky=W, padx=1)
		self.perim = ttk.Label(self.ind1F, text="Perimeter ")
		self.perim.grid(row=1, column=4, sticky=W, padx=1)

		self.sizeTxt = "= " + str(data[0]) + " x " + str(data[1]) + "  "#width and height
		self.sizeP = ttk.Label(self.ind1F, text=self.sizeTxt)
		self.sizeP.grid(row=0, column=1, sticky=W, padx=1)
		
		self.sclTxt = "= 1 : " + str(data[2]) #scaling
		self.sclP = ttk.Label(self.ind1F, text=self.sclTxt)
		self.sclP.grid(row=1, column=1, sticky=W, padx=1)

		self.resoP = ttk.Label(self.ind1F, text="= " + str(data[3])) # reso
		self.resoP.grid(row=2, column=1, sticky=W, padx=1)
		
		self.areaP = ttk.Label(self.ind1F, text="= " + str(round(data[4], 2)) + "  ") #area
		self.areaP.grid(row=0, column=3, sticky=W, padx=1)

		self.lngpthP = ttk.Label(self.ind1F, text="= " + str(data[5])) #long path
		self.lngpthP.grid(row=1, column=3, sticky=W, padx=1)

		self.bndrP = ttk.Label(self.ind1F, text="= " + str(data[6])) #num of boundary point
		self.bndrP.grid(row=2, column=3, sticky=W, padx=1)

		self.numTurnP = ttk.Label(self.ind1F, text="= " + str(data[7])) #num of boundary point
		self.numTurnP.grid(row=0, column=5, sticky=W, padx=1)

		self.perimP = ttk.Label(self.ind1F, text="= " + str(data[8])) #perimeter
		self.perimP.grid(row=1, column=5, sticky=W, padx=1)

	def ind2(self, data2):
		#self.tab2_2_frame5 indikator
		self.ind2F = ttk.LabelFrame(self.tab2_2, text="Indicator")
		self.ind2F.grid(row=0, column=9, sticky=NW, padx=3, pady=3)
		self.size2 = ttk.Label(self.ind2F, text="Size ")
		self.size2.grid(row=0, column=0, sticky=W, padx=1)
		self.scl2 = ttk.Label(self.ind2F, text="Scale ")
		self.scl2.grid(row=1, column=0, sticky=W, padx=1)
		self.reso2 = ttk.Label(self.ind2F, text="Reso ")
		self.reso2.grid(row=2, column=0, sticky=W, padx=1)
		self.area2 = ttk.Label(self.ind2F, text="Area ")
		self.area2.grid(row=0, column=2, sticky=W, padx=1)
		self.lgpth2 = ttk.Label(self.ind2F, text="Long Path ")
		self.lgpth2.grid(row=1, column=2, sticky=W, padx=1)
		self.trng2 = ttk.Label(self.ind2F, text="Boundary ")
		self.trng2.grid(row=2, column=2, sticky=W, padx=1)
		self.bndr2 = ttk.Label(self.ind2F, text="Turning ")
		self.bndr2.grid(row=0, column=4, sticky=W, padx=1)
		self.perim = ttk.Label(self.ind2F, text="Perimeter ")
		self.perim.grid(row=1, column=4, sticky=W, padx=1)

		self.sizeTxt2 = "= " + str(data2[0]) + " x " + str(data2[1]) + "  "#width and height
		self.sizeP2 = ttk.Label(self.ind2F, text=self.sizeTxt2)
		self.sizeP2.grid(row=0, column=1, sticky=W, padx=1)
		
		self.sclTxt2 = "= 1 : " + str(data2[2]) #scaling
		self.sclP2 = ttk.Label(self.ind2F, text=self.sclTxt2)
		self.sclP2.grid(row=1, column=1, sticky=W, padx=1)

		self.resoP2 = ttk.Label(self.ind2F, text="= " + str(data2[3])) # reso
		self.resoP2.grid(row=2, column=1, sticky=W, padx=1)
		
		self.areaP2 = ttk.Label(self.ind2F, text="= " + str(round(data2[4], 2)) + "  ") #area
		self.areaP2.grid(row=0, column=3, sticky=W, padx=1)

		self.lngpthP2 = ttk.Label(self.ind2F, text="= " + str(data2[5])) #long path
		self.lngpthP2.grid(row=1, column=3, sticky=W, padx=1)

		self.bndrP2 = ttk.Label(self.ind2F, text="= " + str(data2[6])) #num of boundary point
		self.bndrP2.grid(row=2, column=3, sticky=W, padx=1)

		self.numTurnP2 = ttk.Label(self.ind2F, text="= " + str(data2[7])) #num of boundary point
		self.numTurnP2.grid(row=0, column=5, sticky=W, padx=1)

		self.perimP = ttk.Label(self.ind2F, text="= " + str(data2[8])) #perimeter
		self.perimP.grid(row=1, column=5, sticky=W, padx=1)


		