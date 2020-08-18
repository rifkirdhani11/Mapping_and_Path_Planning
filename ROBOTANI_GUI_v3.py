import os
import sys
sys.path.append('..')
import math
import time
import numpy as np
import CPplanner, ODMapper, hed_contour, firebaseSend

from serial import *
from tkinter import *
from tkinter.ttk import Progressbar
from tkinter import ttk, filedialog, scrolledtext, messagebox
from PIL import ImageTk,Image 
from attributeGUI_v4 import Attrib
from gpsCoordinate import NewCoordinate
from datetime import datetime

class Robotani:
	def __init__(self, root):
		self.frstPthFlagMan, self.frstPthFlagAuto = 0, 0
		self.x1, self.x2, self.xZ, self.y1, self.y2, self.yZ= 0, 0, 0, 0, 0, 0
		self.pjg, self.pjgTot, self.l, self.j, self.finalPath = 0, 0, 0, 0, 0
		self.realDist, self.realDistTotal = 0, 0
		self.tifFlag = 0

		self.tabFlag = 0
		self.boundaryFlagBtn = IntVar()
		self.boundaryFlagBtn.set(0)
		self.zeroFlag = IntVar()
		self.zeroFlag.set(0)
		self.treshMin = IntVar()
		self.treshMin.set(160)
		self.treshMax = IntVar()
		self.treshMax.set(255)

		self.ox, self.oy, self.poly = [], [], []
		self.scaledOx, self.scaledOy, self.scaledPx, self.scaledPy = [], [], [], []
		self.finalPath, self.finalPathLine, self.displayPath, self.finalPathEd = [], [], [], []
		self.oLat1, self.lat2, self.oLon1, self.lon2, self.oLoc1, self.oLocAll = [], [], [], [], [], []
		self.pLat1, self.pLon1, self.pLoc1 = [], [], []
		self.oxReal, self.oyReal = [], []
		
		self.window = root
		
		self.atr = Attrib()
		#self.geo = NewCoordinate()
		#[width, height, scale, resolution, area, longpath, num boundary, num turning]
		self.dataInd = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		self.dataInd2 = [0, 0, 0, 0, 0, 0, 0, 0, 0]
		self.dataAttrib = []#[self.tab1, self.tab2_1, self.tab2_2, self.rasioIn1, self.rasioIn2, self.treshMin, self.treshMax, self.port1, self.baud1]
		self.dataAttrib = self.atr.tab(self.window, self.open_map, self.reset1, 
									self.viewdata1, self.senddata1, self.simulate1, self.connect1, self.disconnect,
									self.boundaryMaker, self.boundaryMakerStop, self.zeroPositionSet,
									self.open_map2, self.reset2, self.getBoundPoint, self.polygon,
									self.viewdata2, self.senddata2, self.simulate2, self.connect2, self.boundaryFlagBtn, self.zeroFlag,
									self.treshMin, self.treshMax)
		self.atr.ind(self.dataInd)
		self.atr.ind2(self.dataInd2)
		ODMapper.attributeODM(root, self.dataAttrib[0]) #tab1

	def zeroPositionSet(self):
		self.window.config(cursor='plus red red')
	def boundaryMaker(self):
		self.window.config(cursor='plus red red')
	def boundaryMakerStop(self):
		self.window.config(cursor='arrow red red')

	def clickManu(self, event):
		#self.window.config(cursor='arrow red red')
		#global frstPthFlagAuto, x1, x2, xZ, y1, y2, yZ, ox, oy, poly, corner, corner2
		self.canvasImg1 = event.widget
		print ("clicked at ", event.x, event.y)
		self.zeroFlagVal  = self.zeroFlag.get()
		self.boundaryFlag = self.boundaryFlagBtn.get()

		if self.frstPthFlagAuto==1 and self.boundaryFlag==1:
			self.window.config(cursor='plus red red')
			self.x2 = self.canvasImg1.canvasx(event.x)
			self.y2 = self.canvasImg1.canvasy(event.y)
			self.ox.append(round((self.x2 - self.xZ)/self.ratio))
			#oy.append(((abs(imShow.height()-y2))-yZ))
			#self.oy.append(round(abs(self.yZ - self.y2)/self.ratio))
			if self.yZ!=0:
				self.oy.append(round((self.yZ - self.y2)/self.ratio))
			else:
				self.oy.append(round((self.imShow.height()-self.y2)/self.ratio))
			self.poly.append(self.x2)
			self.poly.append(self.y2)
			self.corner2 = np.array((self.x2/100, self.y2/100))
			self.corner = np.vstack((self.corner, self.corner2))
			
			if self.tifFlag == 1 :
				self.realX = (self.x2 - self.xZ)/self.ratio
				self.realY = abs(self.yZ - self.y2)/self.ratio
				self.oLoc1 = self.geo1.getLoc(self.realX, self.realY)
				self.oLat1.append(self.oLoc1[0])
				self.oLon1.append(self.oLoc1[1])
				self.oLocAll.append(self.oLoc1)

			self.canvasImg1.create_oval(self.x2-5, self.y2-5, self.x2+5, self.y2+5, outline="#f11", fill="#fff", width=2)
			self.canvasImg1.create_line(self.x1,self.y1,self.x2,self.y2, fill="#0f0", width=3)

			self.x1 = self.x2
			self.y1 = self.y2
		
		elif self.frstPthFlagAuto==0 and self.boundaryFlag==1:
			self.window.config(cursor='plus red red')
			self.x1 = self.canvasImg1.canvasx(event.x)
			self.y1 = self.canvasImg1.canvasy(event.y)
			
			self.ox = [round((self.x1 - self.xZ)/self.ratio)]
			if self.yZ!=0:
				self.oy = [round((self.yZ - self.y1)/self.ratio)]
			else:
				self.oy = [round((self.imShow.height()-self.y1)/self.ratio)]
			#self.oy = [((self.imShow.heigtht()-self.y1) - (self.imShow.heigtht()-self.yZ))/self.ratio]
			self.poly = [self.x1, self.y1]
			self.corner = np.array((self.x1/100, self.y1/100))
			
			self.canvasImg1.create_oval(self.x1-5, self.y1-5, self.x1+5, self.y1+5, outline="#f11", fill="#fff", width=2)
			
			if self.tifFlag == 1:
				self.realX = (self.x1 - self.xZ)/self.ratio
				self.realY = abs(self.yZ - self.y1)/self.ratio
				self.oLoc1 = self.geo1.getLoc(self.realX, self.realY)
				self.oLat1.append(self.oLoc1[0])
				self.oLon1.append(self.oLoc1[1])
				self.oLocAll.append(self.oLoc1)
				#print(self.loc1)

			self.frstPthFlagAuto = 1

		elif self.zeroFlagVal==1: #Set zero point
			self.window.config(cursor='plus red red')
			self.xZ = self.canvasImg1.canvasx(event.x)
			self.yZ = self.canvasImg1.canvasy(event.y)
		
			#point indicator
			self.canvasImg1.create_rectangle(self.xZ-5, self.yZ-12, self.xZ+5, self.yZ+12, outline="#f11", fill="#f11", width=2) 
			self.canvasImg1.create_rectangle(self.xZ-12, self.yZ-5, self.xZ+12, self.yZ+5, outline="#f11", fill="#f11", width=2) 
			#line edges
			self.canvasImg1.create_line(self.xZ, 0, self.xZ, self.imShow.height(), dash=(4,2), fill="#f22", width=2)
			self.canvasImg1.create_line(0, self.yZ, self.imShow.width(), self.yZ, dash=(4,2), fill="#f22", width=2)

	def clickZero(self):
		self.canvasImg2 = event.widget
		print ("clicked at ", event.x, event.y)

		self.window.config(cursor='plus red red')
		self.xZ = self.canvasImg2.canvasx(event.x)
		self.yZ = self.canvasImg2.canvasy(event.y)
		
		#point indicator
		self.canvasImg2.create_rectangle(self.xZ-5, self.yZ-12, self.xZ+5, self.yZ+12, outline="#f11", fill="#f11", width=2) 
		self.canvasImg2.create_rectangle(self.xZ-12, self.yZ-5, self.xZ+12, self.yZ+5, outline="#f11", fill="#f11", width=2) 
		#line edges
		self.canvasImg2.create_line(self.xZ, 0, self.xZ, self.imShow2.height(), dash=(7,7), fill="#f22", width=1)
		self.canvasImg2.create_line(0, self.yZ, self.imShow2.width(), self.yZ, dash=(7,7), fill="#f22", width=1)

	def polygon(self):
		#global finalPath, ltot
		self.frstPthFlagAuto = 0
		self.ltot = 0
		self.logName = 'Coordinate-Auto.txt' #% (str(datetime.now()))
		self.f = open(self.logName, "w+")
		'''
		#====================== For Testing Only ===========================
		self.tabFlag = 1
		self.ox = [8, 14, 33, 65, 111, 141, 159, 195, 205, 236, 252, 256, 238, 239, 257, 278, 213, 201, 182, 125, 96]
		self.oy = [51, 103, 115, 115, 126, 121, 113, 118, 116, 129, 145, 133, 86, 43, 24, 13, 11, 14, 8, 24, 23]
		for i in range(len(self.ox)-1):
			self.poly.append(self.ox[i])
			self.poly.append(self.oy[i])
		#====================== Dont Forget to Comment =====================
		'''
		#Push data to indicator
		self.dataInd[6] = len(self.ox) #push number of boundary point to ind
		self.dataInd2[6] = len(self.ox) #push number of boundary point to ind
		
		if self.tabFlag==1:
			self.reso = int(self.dataAttrib[3].get())
			self.canvasImg1.create_line(self.poly, self.poly[0], self.poly[1], fill="#0f0", width=5) 
		elif self.tabFlag==2:
			self.reso = int(self.dataAttrib[4].get()) 

		#Run path planner
		self.ox.append(self.ox[0])
		self.oy.append(self.oy[0])
		print("ox =", self.ox)
		print("oy =", self.oy)
		self.px, self.py, self.pl, self.turn = CPplanner.planning_animation(self.ox, self.oy, self.reso) #start CPP
		print("px =", self.px)
		print("py =", self.py)
		#Check type of image
		if self.tifFlag == 1:
			for i in range(len(self.px)):
				if self.tabFlag==1:
					self.realPx = int(self.px[i]) #(self.px[i] - self.xZ)/self.ratio
					self.realPy = int(self.img.size[1]-self.py[i]) #abs(self.yZ - self.py[i])/self.ratio
				else:
					self.realPx = int(self.px[i])
					self.realPy = int(self.img2.size[1]-self.py[i])

				self.pLoc1 = self.geo1.getLoc(self.realPx, self.realPy)
				self.pLat1.append(self.pLoc1[0])
				self.pLon1.append(self.pLoc1[1])
		
			#Get Distance path
			for i in(range(len(self.pLat1)-1)):
				self.realDist = self.geo1.getDist(self.pLat1[i], self.pLat1[i+1], self.pLon1[i], self.pLon1[i+1])
				self.realDistTotal += self.realDist
				#print(self.realDist)
			print("boundary", self.oLat1, self.oLon1)
			print("path",self.pLat1, self.pLon1)	
			print("Total Distance Path", self.realDistTotal)
			print("lat lon result", self.pLat1, self.pLon1)

		for i in range(len(self.ox)):
			self.scaledOx.append(self.ox[i]*0.0284) #konstanta GSDx
			self.scaledOy.append(self.oy[i]*0.021) #konstanta GSDy

		self.perim = round(CPplanner.perimeterArea(self.scaledOx, self.scaledOy),2)
		self.area = round(CPplanner.polygonArea(self.scaledOx, self.scaledOy, len(self.ox)),2) #*2.3**2), 2) # / self.reso**2 #Luas berdasarkan resolusi

		#Put data to indicator
		self.dataInd[4] = self.area #push area to ind
		self.dataInd2[4] = self.area #push area to ind
		self.dataInd[3] = self.reso #push reso to ind
		self.dataInd2[3] = self.reso #push reso to ind
		self.dataInd[7] = self.turn #push number of turning to ind
		self.dataInd2[7] = self.turn #push number of turning to ind
		self.dataInd[8] = self.perim #push perimeter to ind
		self.dataInd2[8] = self.perim #push perimeter to ind

		self.n = len(self.py)
		
		for i in range(len(self.px)):
			self.scaledPx.append(self.px[i]*0.0284) #konstanta GSDx
			self.scaledPy.append(self.py[i]*0.021) #konstanta GSDy

		for i in range(self.n-1): #loging data coordinates
			#Distance form pixels
			self.ln = math.sqrt((math.pow((self.scaledPx[i+1]-self.scaledPx[i]),2)+(math.pow((self.scaledPy[i+1]-self.scaledPy[i]),2))))
			self.ltot += self.ln 
			#Record data on txt
			self.f.write("{%d, %d}," % (self.px[i], self.py[i]))
			#Collect data for display and database
			self.finalPath.append(round(self.px[i], 2))
			if self.tabFlag==2:
				self.finalPath.append(round(abs(self.py[i]- self.img2R.size[1]), 2))
			else:
				self.finalPath.append(round(abs(self.py[i]), 2))

			self.finalPathLine.append(int((self.px[i] + self.xZ)*self.ratio))
			
			if self.yZ!=0:
				self.finalPathLine.append(int((self.yZ - self.py[i])*self.ratio))
			else:
				if self.tabFlag == 1:
					self.finalPathLine.append(int((self.img.size[1]-self.py[i])* self.ratio))
				else:
					self.finalPathLine.append(int((self.img2.size[1]-self.py[i])*self.ratio2))

			if self.tabFlag==2:
				#Collect data special edge detection
				self.finalPathEd.append(int(self.px[i]*self.ratio2))#+ abs(self.img2.size[0]-self.basewidth))
				self.finalPathEd.append(int((self.img2.size[1]-self.py[i])* self.ratio2))#+ abs(self.img2.size[1]-self.hsize))

		#print("real distance", self.ltot/self.reso)
		self.dataInd[5] = round(self.ltot,2)#/self.reso, 2) #push long path to ind
		self.dataInd2[5] = round(self.ltot,2)#/self.reso, 2) #push long path to ind

		if self.tabFlag==1:
			self.atr.ind(self.dataInd)
			#print("path line manu :",self.finalPathLine)
			self.canvasImg1.create_line(self.finalPathLine, fill='#f00', width=5) 
		elif self.tabFlag==2:
			self.atr.ind2(self.dataInd2)
			for i in range(len(self.finalPath)):
				self.displayPath.append(int(self.finalPathEd[i]))# * (1/self.ratio2))
			self.canvasImg2ori.create_line(self.displayPath, fill='#f00', width=5)
		print(self.displayPath)
		
		if self.tifFlag==1:
			self.geo1.gmPlot(self.oLat1, self.oLon1, self.pLat1, self.pLon1) #polygon
		self.f.close()

	def open_map(self):
		self.filemap1 = filedialog.askopenfilename(initialdir = "Data_Pengujian")
		self.tabFlag = 1
		
		self.ext = os.path.splitext(self.filemap1)[-1].lower()
		if self.ext == '.tif' :
			self.geo1 = NewCoordinate(self.filemap1)
			self.tifFlag = 1
		#print("tifFlag =", self.tifFlag)
			
		self.img = Image.open(self.filemap1)

		self.baseheight = 530
		self.ratio = (self.baseheight/float(self.img.size[1]))
		self.wsize = int((float(self.img.size[0])*float(self.ratio)))
		self.imgR = self.img.resize((self.wsize, self.baseheight), Image.ANTIALIAS)
		self.imShow = ImageTk.PhotoImage(self.imgR)
		
		self.canvasImg1 = self.atr.canvas1(self.dataAttrib[1], self.imShow, self.clickManu)

		self.dataInd[0] = self.img.size[0]
		self.dataInd[1] = self.img.size[1]
		self.dataInd[2] = round(1/self.ratio, 2)
		self.atr.ind(self.dataInd)

	def reset1(self):
		self.canvasImg1.delete("all")

		self.frstPthFlagMan, self.frstPthFlagAuto = 0, 0
		self.x1, self.x2, self.xZ, self.y1, self.y2, self.yZ= 0, 0, 0, 0, 0, 0
		self.pjg, self.pjgTot, self.l, self.j, self.finalPath = 0, 0, 0, 0, 0
		self.realDist, self.realDistTotal = 0, 0
		self.tifFlag, self.tabFlag = 0, 0

		self.ox, self.oy, self.px, self.py, self.poly = [], [], [], [], []
		self.finalPath, self.finalPathLine, self.displayPath, self.finalPathEd = [], [], [], []
		self.oLat1, self.lat2, self.oLon1, self.lon2, self.oLoc1, self.oLocAll = [], [], [], [], [], []
		self.pLat1, self.pLon1, self.pLoc1 = [], [], []
		self.oxReal, self.oyReal = [], []

		#[width, height, scale, resolution, area, longpath, num boundary, num turning]
		self.dataInd = [0, 0, 0, 0, 0, 0, 0, 0]
		self.atr.ind(self.dataInd)

	def open_map2(self):
		self.filemap2 = filedialog.askopenfilename(initialdir = "Data_Pengujian")
		self.tabFlag = 2

		self.ext = os.path.splitext(self.filemap2)[-1].lower()
		if self.ext == '.tif' :
			self.geo1 = NewCoordinate(self.filemap2)
			self.tifFlag = 1
		#print("tifFlag =", self.tifFlag)

		self.img2ori = Image.open(self.filemap2)
		self.img2 = hed_contour.edRun(self.filemap2)
		
		#self.baseheight = 530
		self.basewidth = 500
		#self.ratio = (self.baseheight/float(self.img2.size[1]))
		self.ratio2 = (self.basewidth/float(self.img2.size[0]))

		#self.wsize = int((float(self.img2.size[0])*float(self.ratio)))
		self.hsize = int((float(self.img2.size[1])*float(self.ratio2)))
		#self.img2 = self.img2.resize((self.wsize, self.baseheight), Image.ANTIALIAS)
		self.img2oriR = self.img2ori.resize((self.basewidth, self.hsize), Image.ANTIALIAS)
		self.img2R = self.img2.resize((self.basewidth, self.hsize), Image.ANTIALIAS)
		self.imShow2ori = ImageTk.PhotoImage(self.img2oriR)
		self.imShow2 = ImageTk.PhotoImage(self.img2R)
		#display to imout
		self.canvasImg2ori, self.canvasImg2, self.putImageOri, self.putImage = self.atr.canvas2(self.dataAttrib[2], self.imShow2ori, self.imShow2, self.clickZero)
		#display to indicator
		
		self.dataInd2[0] = self.img2R.size[0]
		self.dataInd2[1] = self.img2R.size[1]
		self.dataInd2[2] = round(1/self.ratio2, 2)
		self.atr.ind2(self.dataInd2)

	def reset2(self):
		self.canvasImg2.delete("all")
		self.canvasImg2ori.delete("all")

		self.pjg, self.pjgTot, self.l, self.j, self.finalPath = 0, 0, 0, 0, 0
		self.realDist, self.realDistTotal = 0, 0
		self.tifFlag, self.tabFlag = 0, 0

		self.ox, self.oy, self.px, self.py, self.poly = [], [], [], [], []
		self.finalPath, self.finalPathLine, self.displayPath, self.finalPathEd = [], [], [], []
		self.oLat1, self.lat2, self.oLon1, self.lon2, self.oLoc1, self.oLocAll = [], [], [], [], [], []
		self.pLat1, self.pLon1, self.pLoc1 = [], [], []
		self.oxReal, self.oyReal = [], []

		#[width, height, scale, resolution, area, longpath, num boundary, num turning]
		self.dataInd2 = [0, 0, 0, 0, 0, 0, 0, 0]
		self.atr.ind2(self.dataInd2)
		
	def getBoundPoint(self):
		tMin = self.treshMin.get()
		tMax = self.treshMax.get()
		self.ox, self.oy, self.poly = hed_contour.cntrGet(tMin, tMax)

		for i in range(len(self.ox)-1):
			self.poly.append(int(self.ox[i]))
			self.poly.append(int(self.oy[i]))

		#display
		self.imgC = Image.open("contour_result.jpg")
		'''
		self.ratio = (self.baseheight/float(self.imgC.size[1]))
		self.wsize = int((float(self.imgC.size[0])*float(self.ratio)))
		'''
		self.basewidth = 500
		#self.ratio = (self.baseheight/float(self.img2.size[1]))
		self.ratio = (self.basewidth/float(self.img2.size[0]))
		#self.wsize = int((float(self.img2.size[0])*float(self.ratio)))
		self.hsize = int((float(self.img2.size[1])*float(self.ratio)))
		self.imgC = self.imgC.resize((self.basewidth, self.hsize), Image.ANTIALIAS)
		self.imShowNew = ImageTk.PhotoImage(self.imgC)
		self.dataInd2[0] = self.imgC.size[0]
		self.dataInd2[1] = self.imgC.size[1]
		self.dataInd2[2] = round(1/self.ratio, 2)
		self.atr.ind2(self.dataInd2)
		#display contour detected
		self.canvasImg2.itemconfig(self.putImage, image=self.imShowNew)

	def viewdata1(self):
		self.view1 = Toplevel()
		self.view1.title("Path Coordinate Viewer")
		self.view1.geometry("300x300+500+200")
		self.path1 = scrolledtext.ScrolledText(self.view1, width=500, height=200)
		j = 0
		
		if self.tifFlag == 1 :
			for i in range(len(self.pLat1)):
				self.viewText = '[%s] %f , %f \n' %(j, self.pLat1[i], self.pLon1[i])
				self.path1.insert(INSERT, self.viewText)
				j+=1
		else:
			for i in range(len(self.finalPath)):
				if i%2 == 0:
					self.viewText = '[%s] %d , %d \n' %(j, self.finalPath[i], self.finalPath[i+1])
					self.path1.insert(INSERT, self.viewText)
					j+=1
		self.path1.pack()

	def senddata1(self):
		j = 0
		if self.tifFlag == 1 :
			try:
				messagebox.showwarning('ROBOTANI', 'Sending data! \n Please Wait..')
				for i in range(len(self.pLat1)):
					#self.viewText = '[%s] %f , %f \n' %(j, self.pLat1[i], self.pLon1[i])
					self.viewText = '*%sx%fy %f_' %(j, self.pLat1[i], self.pLon1[i])
					self.tfData = str.encode(self.viewText)
					self.serial_object.write(self.tfData)
					time.sleep(0.3)
					j+=1
				messagebox.showinfo('ROBOTANI', 'Path Sent!')
			except:
				messagebox.showerror('ROBOTANI', 'Sent Failed! \n Check the connection!')
		else:
			try:
				messagebox.showwarning('ROBOTANI', 'Sending data! \n Please Wait..')
				for i in range(len(self.finalPath)):
					if i%2 == 0:
						#self.viewText = '[%s] %d , %d \n' %(j, self.finalPath[i], self.finalPath[i+1])
						self.viewText = '*%sx%dy%d_' %(j, self.finalPath[i], self.finalPath[i+1])
						self.tfData = str.encode(self.viewText)
						self.serial_object.write(self.tfData)
						time.sleep(0.3)
						j+=1
				messagebox.showinfo('ROBOTANI', 'Path Sent!')
			except:
				messagebox.showerror('ROBOTANI', 'Sent Failed! \n Check the connection!')
		firebaseSend.main(str(self.finalPath), 0) #0 - manual, 1 - auto
		

	def connect1(self):
		self.port = self.dataAttrib[7].get()
		self.baud = self.dataAttrib[8].get()
		try:
			self.serial_object = Serial('COM'+str(self.port), self.baud)
			self.dataAttrib[11].configure(bg='green') #connection indicator
			messagebox.showinfo('ROBOTANI', 'Connected!')
		except:
			messagebox.showerror('ROBOTANI', 'Connection Failed!')
			self.dataAttrib[11].configure(bg='red') #connection indicator
		print(self.port + " " + self.baud)
	
	def disconnect(self):
		self.serial_object.close()
		self.dataAttrib[11].configure(bg='red')

	def simulate1(self):
		self.reso = int(self.dataAttrib[3].get()) 
		CPplanner.simulate(self.corner, self.reso)

	def viewdata2(self):
		self.view2 = Toplevel()
		self.view2.title("Path Coordinate Viewer")
		self.view2.geometry("300x300+500+200")
		self.path2 = scrolledtext.ScrolledText(self.view2, width=500, height=200)
		j = 0
		if self.tifFlag == 1 :
			for i in range(len(self.pLat1)):
				viewText = '[%s] %f , %f \n' %(j, self.pLat1[i], self.pLon1[i])
				self.path2.insert(INSERT, viewText)
				j+=1
		else:
			for i in range(len(self.finalPath)):
				if i%2 == 0:
					viewText = '[%s] %d , %d \n' %(j, self.finalPath[i], self.finalPath[i+1])
					self.path2.insert(INSERT, viewText)
					j+=1
		self.path2.pack()

	def senddata2(self):
		self.tfData2 = str.encode(self.viewText + '\n')
		firebaseSend.main(str(self.finalPath), 1) #0 - manual, 1 - auto
		self.serial_object.write(self.tfData2)

		j = 0
		if self.tifFlag == 1 :
			try:
				messagebox.showwarning('ROBOTANI', 'Sending data! \n Please Wait..')
				for i in range(len(self.pLat1)):
					#self.viewText = '[%s] %f , %f \n' %(j, self.pLat1[i], self.pLon1[i])
					self.viewText = '*%s:%f:%f_' %(j, self.pLat1[i], self.pLon1[i])
					self.tfData = str.encode(self.viewText)
					self.serial_object.write(self.tfData)
					time.sleep(0.3)
					j+=1
				messagebox.showinfo('ROBOTANI', 'Path Sent!')
			except:
				messagebox.showerror('ROBOTANI', 'Sent Failed! \n Check the connection!')
		else:
			try:
				messagebox.showinfo('ROBOTANI', 'Sending data! \n Please Wait..')
				for i in range(len(self.finalPath)):
					if i%2 == 0:
						#self.viewText = '[%s] %d , %d \n' %(j, self.finalPath[i], self.finalPath[i+1])
						self.viewText = '*%s:%d:%d_' %(j, self.finalPath[i], self.finalPath[i+1])
						self.tfData = str.encode(self.viewText)
						self.serial_object.write(self.tfData)
						time.sleep(0.3)
						j+=1
			except:
				messagebox.showerror('ROBOTANI', 'Sent Failed! \n Check the connection!')
		firebaseSend.main(str(self.finalPath), 0) #0 - manual, 1 - auto

	def connect2(self):
		self.port = self.dataAttrib[9].get()
		self.baud = self.dataAttrib[10].get()
		try:
			self.serial_object = Serial('COM'+str(self.port), self.baud)
			messagebox.showinfo('ROBOTANI', 'Connected!')
			self.dataAttrib[12].configure(bg='green') #connection indicator
		except:
			messagebox.showerror('ROBOTANI', 'Connection Failed!')
			self.dataAttrib[12].configure(bg='red') #connection indicator
		print(self.port + " " + self.baud)

	def simulate2(self):
		self.reso = int(self.dataAttrib[3].get()) 
		CPplanner.simulate(self.corner, self.ox, self.oy)

def main():
	root = Tk()
	root.iconbitmap('pens.ico')
	root.title("ROBOTANI Mapping and Path Planning")
	root.geometry('850x350')
	root.config(cursor='arrow red red')

	s = ttk.Style()
	s.configure('TNotebook.Tab', font=('Segoe UI','10'))

	runRobotain = Robotani(root)
	root.mainloop()

if __name__ == "__main__":
	main()

	

	