from osgeo import gdal, osr
from math import radians, cos, sin, asin, sqrt 
import gmplot

filepath = 'D:/MECHATRONICS ENGINEERING/Bismillah Proyek Akhir/Mapping_and_Path_Planning/Data_Pengujian/data_alun-alun_lamongan/odm_orthophoto/odm_orthophoto.tif'

class NewCoordinate:
	def __init__(self, file):
		self.lat = 0
		self.lon = 0
		#self.filepath = file
		self.ds = gdal.Open(file)

	def getLoc(self, mx, my): #mx,my coordinate from pixels
		self.gt = self.ds.GetGeoTransform()
		#print(self.gt)
		x_min = self.gt[0] 
		x_size = self.gt[1] #width img
		y_min = self.gt[3]
		y_size = self.gt[5] #height img

		# mx, my = 0000, 0000  #coord in map units, as in question
		px = mx * x_size + x_min #x pixel
		py = my * y_size + y_min #y pixel
		#print("px py :",px,py)

		proj = self.ds.GetProjectionRef()
		#print("proj =", proj)
		# get CRS from dataset 
		crs = osr.SpatialReference()
		crs.ImportFromWkt(proj)
		# create lat/long crs with WGS84 datum
		crsGeo = osr.SpatialReference()
		crsGeo.ImportFromEPSG(4326) # 4326 is the EPSG id of lat/long crs 
		t = osr.CoordinateTransformation(crs, crsGeo)
		(self.lat, self.lon, z) = t.TransformPoint(px, py)

		return self.lat, self.lon

	def getDist(self, lat1, lat2, lon1, lon2): 
		# The math module contains a function named 
		# radians which converts from degrees to radians. 
		lon1 = radians(lon1) 
		lon2 = radians(lon2) 
		lat1 = radians(lat1) 
		lat2 = radians(lat2) 
		
		# Haversine formula 
		dlon = lon2 - lon1 
		dlat = lat2 - lat1 
		a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2

		c = 2 * asin(sqrt(a)) 
		
		# Radius of earth in kilometers. Use 3956 for miles 
		r = 6371

		dist = c*r #in km
		dist = round((dist * 1000), 2) #in meter
		
		# calculate the result 
		return dist 
	
	def gmPlot(self, olat, olon, plat, plon):
		gmap5 = gmplot.GoogleMapPlotter(olat[0], olon[0], 17) 
		gmap5.scatter( olat, olon, '#FF0000', size = 1, marker = False) 

		gmap5.plot(plat, plon, size=3, color = 'red')
		gmap5.polygon( olat, olon, size=5, color = 'cornflowerblue') 
		
		gmap5.draw( "map-out.html" ) 

if __name__ == "__main__":
	newCrdnt = NewCoordinate(filepath)
	lat1, lon1 = newCrdnt.getLoc(0, 0)
	lat2, lon2 = newCrdnt.getLoc(0, 100)
	dist = newCrdnt.getDist(lat1, lat2, lon1, lon2)
	print("coordinate1", lat1, lon1)
	print("coordinate2", lat2, lon2)
	print("distance", dist)
	pass
