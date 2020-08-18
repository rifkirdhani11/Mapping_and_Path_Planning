# import the necessary packages
import cv2
import os
import numpy as np
import imutils
from PIL import Image
from skimage import measure

Area = []
arCntSize = []

class CropLayer(object):
	def __init__(self, params, blobs):
		# initialize our starting and ending (x, y)-coordinates of
		# the crop
		self.startX = 0
		self.startY = 0
		self.endX = 0
		self.endY = 0

	def getMemoryShapes(self, inputs):
		# the crop layer will receive two inputs -- we need to crop
		# the first input blob to match the shape of the second one,
		# keeping the batch size and number of channels
		(inputShape, targetShape) = (inputs[0], inputs[1])
		(batchSize, numChannels) = (inputShape[0], inputShape[1])
		(H, W) = (targetShape[2], targetShape[3])

		# compute the starting and ending crop coordinates
		self.startX = int((inputShape[3] - targetShape[3]) / 2)
		self.startY = int((inputShape[2] - targetShape[2]) / 2)
		self.endX = self.startX + W
		self.endY = self.startY + H

		# return the shape of the volume (we'll perform the actual
		# crop during the forward pass
		return [[batchSize, numChannels, H, W]]

	def forward(self, inputs):
		# use the derived (x, y)-coordinates to perform the crop
		return [inputs[0][:, :, self.startY:self.endY,
				self.startX:self.endX]]

def edRun(imInput):
	# load our serialized edge detector from disk
	print("[INFO] loading edge detector...")
	protoPath = os.path.sep.join(["hed_model/",
		"deploy.prototxt"])
	modelPath = os.path.sep.join(["hed_model/",
		"hed_pretrained_bsds.caffemodel"])
	net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

	# register our new layer with the model
	cv2.dnn_registerLayer("Crop", CropLayer)

	# load the input image and grab its dimensions
	image = cv2.imread(imInput) #from GUI
	(H, W) = image.shape[:2]
	
	# construct a blob out of the input image for the Holistically-Nested
	# Edge Detector
	#blob = cv2.dnn.blobFromImage(image, scalefactor=1.0, size=(W, H),
	#	mean=(104.00698793, 116.66876762, 122.67891434),
	#	swapRB=False, crop=False)
	blob = cv2.dnn.blobFromImage(image, scalefactor=1.0, size=(W, H),
		mean=(104.00698793, 116.66876762, 122.67891434),
		swapRB=False, crop=False)

	# set the blob as the input to the network and perform a forward pass
	# to compute the edges
	print("[INFO] performing holistically-nested edge detection...")
	net.setInput(blob)
	hed = net.forward()
	hed = cv2.resize(hed[0, 0], (W, H))
	hed = (255 * hed).astype("uint8")
	
	print(hed.shape)

	new_hed = Image.fromarray(hed)
	new_hed.save("hed_result.jpg")
	'''
	#for test only
	cv2.imshow('input', image) 
	cv2.imshow('output', hed)
	cv2.waitKey(0)
	'''
	return new_hed

#filepath = ("D:/MECHATRONICS ENGINEERING/Bismillah Proyek Akhir/sawah3.jpg")
#run(filepath)

def cntrGet(treshMin, treshMax):
	im_in = cv2.imread("hed_result.jpg")
	'''
	basewidth = 500
	ratio = (basewidth/float(im_in.shape[0]))
	hsize = int((float(im_in.shape[1])*float(ratio)))
	dim = (basewidth, hsize)
	im_in_resized = cv2.resize(im_in, dim)
	'''
	gray = cv2.cvtColor(im_in, cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	im_th = cv2.threshold(blurred, treshMin, treshMax, cv2.THRESH_BINARY)[1]#160 min, 255 max

	# Copy the thresholded image.
	im_floodfill = im_th.copy()

	# Mask used to flood filling.
	# Notice the size needs to be 2 pixels than the image.
	h, w = im_th.shape[:2]
	mask = np.zeros((h+2, w+2), np.uint8)

	# Floodfill from point (0, 0)
	cv2.floodFill(im_floodfill, mask, (0,0), 255)

	# Invert floodfilled image
	im_floodfill_inv = cv2.bitwise_not(im_floodfill)

	#Find the contour
	print("[INFO] Finding Contour")
	cnts = cv2.findContours(im_floodfill_inv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	#Collect all of size of contour
	ncnts = len(cnts)
	for i in range(ncnts-1):
		contourCdnt = cnts[i]
		cntSize = len(contourCdnt)
		arCntSize.append(cntSize)

	print("The number of area =", ncnts)
	print("The index largest area =", np.argmax(arCntSize)) #position of the larger area
	print("The largest area =", np.max(arCntSize))

	#Draw contour of target area
	print("[INFO] Drawing Contour")
	cntTarget = cnts[np.argmax(arCntSize)] #contour of target area (max area)
	cv2.drawContours(im_in, [cntTarget], -1, (0, 255, 0), 2)
	cv2.imwrite("contour_result.jpg", im_in)
	nc = len(cntTarget)

	#Collect the contour coordinate
	ox = [int(cntTarget[0,0,0])]
	oy = [int(im_in.shape[0]-cntTarget[0,0,1])]
	poly = [ox, oy]
	for i in range(0,nc-1):
		ox.append(int(cntTarget[i+1,0,0]))
		oy.append(int(im_in.shape[0]-cntTarget[i+1,0,1]))
		poly.append(ox)
		poly.append(oy)
	print("[INFO] Boundary Point Generated")
	print("The number of boundary point =", len(ox))
	print("image height =", im_in.shape[0])
	'''
	#for test only
	cv2.imshow('input', im_in) 
	#cv2.imshow('output', hed)
	cv2.waitKey(0)
	'''
	return ox, oy, poly
'''
filemap = "D:/MECHATRONICS ENGINEERING/Bismillah Proyek Akhir/Mapping_and_Path_Planning/Rice_Field(5)_v2_cropped.jpg"
#im_ed = edRun(filemap)
im_cd = cntrGet(80, 255)
'''