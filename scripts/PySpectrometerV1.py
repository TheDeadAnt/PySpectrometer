import tkinter
import tkinter.font as tkFont
import cv2
import PIL
from PIL import ImageTk, Image
import PIL.Image
import PIL.ImageTk
import time
import numpy as np
from scipy.signal import savgol_filter
import peakutils
import base64
import gpiozero
#import RPi.GPIO as GPIO
#from gpiozero.pins.mock import MockFactory
#gpiozero.Device.pin_factory = MockFactory()
from gpiozero import LED
from time import sleep
from collections import deque
import argparse
from functools import partial
import multiprocessing
from multiprocess import Process, Queue
#import PyMata


#load = Image.open(r'/home/pi/Desktop/spectrum.png')
load = Image.open(r'C:/Users/Douge/Pictures/spectrum.jfif')



class App:
	DEFAULT_CALIBRATION = ((72, 405), (304, 532))

	def __init__(self, window, graphdata, video_source=0):
		#self.camera = multiprocessing.Process(target=__init__)
		self.window = window

		self.window.geometry('660x570')
		self.window.resizable(width=True, height=False)
		self.window.title("Pyspectrometer")
		self.video_source = video_source
		self.def_font = tkinter.font.nametofont("TkDefaultFont")
		self.def_font.config(size=9)
		self.vid = MyVideoCapture(self.video_source)

		def peakwidth(event):
			# set object value when peakwidth slider moved.
			setattr(self.vid, 'mindist', event)

		def peakthresh(event):
			# set object value when threshold slider moved.
			setattr(self.vid, 'thresh', event)

		def savfilter(event):
			# set object value when threshold slider moved.
			setattr(self.vid, 'savpoly', event)

		def snapshot():
			# Get a frame from the graph, and write it to disk
			ret, graphdata = self.vid.get_graph()
			if ret:
				now = time.strftime("%d-%m-%Y-%H:%M:%S")
				cv2.imwrite("spectrum-" + now + ".jpg",
				            cv2.cvtColor(graphdata[0], cv2.COLOR_RGB2BGR))
				#print(graphdata[1]) #wavelengths
				#print(graphdata[2]) #intensities
				f = open(now+'.csv', 'w')
				f.write('Wavelength,Intensity\r\n')
				for x in zip(graphdata[1], graphdata[2]):
					f.write(str(x[0])+','+str(x[1])+'\r\n')
				f.close()

		def peakhold():
			if self.peakholdbtn.cget("bg") == 'yellow':
				self.peakholdbtn.configure(
					fg="yellow", bg="red", activebackground='red', activeforeground="yellow")
				setattr(self.vid, 'holdpeaks', True)  # set holdpeaks true
				self.filt.configure(state="disabled")
			else:
				self.peakholdbtn.configure(
					fg="black", bg="yellow", activebackground='yellow', activeforeground="black")
				setattr(self.vid, 'holdpeaks', False)  # set holdpeaks true
				self.filt.configure(state="active")




		#self.leds = multiprocessing.Process(target=calibrate)
		#self.leds.start()
		#self.leds.join()

		# Create frames
		self.top_frame = tkinter.Frame(window, width=640, height=240)
		self.top_frame.grid(row=0, column=0, padx=10, pady=5)
		self.bottom_frame = tkinter.Frame(window, width=640, height=255)
		self.bottom_frame.grid(row=1, column=0, padx=10, pady=5)

		#put elements in the frames
		#control frame within frame
		self.control_frame = tkinter.Frame(self.top_frame, width=324)
		self.control_frame.grid(row=0, column=0, padx=0, pady=0)
		self.decoration = tkinter.Canvas(self.control_frame, width=300, height=146, borderwidth=2, relief="raised")
		self.decoration.grid(row=0, column=0, columnspan=3, padx=0, pady=0)

		#control panel items:
		# load the  image file
		self.decorate = ImageTk.PhotoImage(load)
		self.decoration.create_image(2, 2, image=self.decorate, anchor=tkinter.NW)

		#calibrate button
		#self.calbutton = tkinter.Button(self.control_frame, text="Calibrate", width=6, fg="yellow",bg="red", activebackground='red', command=slef.leds)#partial(leds, self, graphdata))
		#self.calbutton.grid(row=4, column=0)

		# Create a canvas that can fit the above video source size
		#weird, we have to sub 3 pixels to have it fit?
		self.canvas0 = tkinter.Canvas(
			self.top_frame, width=317, height=238, borderwidth=2, relief="sunken")
		self.canvas0.grid(row=0, column=2, padx=(10, 0))

		#botttom canvas
		# Create a canvas that can fit the above video source size
		#weird, we have to sub 3 pixels to have it fit?
		self.canvas1 = tkinter.Canvas(self.bottom_frame, width=634,height=252, borderwidth=2, relief="sunken", cursor="tcross")
		self.canvas1.grid(row=0, column=0, padx=0, pady=0, columnspan=5)

		#peaks label
		self.lbpeak = tkinter.Label(self.bottom_frame, text="Peak Width:")
		self.lbpeak.grid(row=1, column=0, pady=20, sticky="nw")

		#slider for peak width
		self.peakwidth = tkinter.Scale(self.bottom_frame, from_=0, to=100, orient="horizontal", command=peakwidth)
		self.peakwidth.grid(row=1, column=1, padx=0, pady=2, sticky="n")
		self.peakwidth.set(50)

		#threshold label
		self.lbthresh = tkinter.Label(self.bottom_frame, text="Threshold:")
		self.lbthresh.grid(row=1, column=2, pady=20, sticky="n")

		#slider for threshold
		self.thresh = tkinter.Scale(self.bottom_frame, from_=0, to=100, orient="horizontal", command=peakthresh)
		self.thresh.grid(row=1, column=3, padx=0, pady=2, sticky="n")
		self.thresh.set(20)

		#Filter label
		self.lbfilt = tkinter.Label(self.bottom_frame, text="Filter:")
		self.lbfilt.grid(row=1, column=4, pady=20, sticky="n")

		#Slider for filter
		self.filt = tkinter.Scale(self.bottom_frame, from_=0,to=16, orient="horizontal", command=savfilter)
		self.filt.grid(row=1, column=5, padx=0, pady=2, sticky="n")
		self.filt.set(7)

		#Peak hold
		self.peakholdbtn = tkinter.Button(self.bottom_frame, text="Peak Hold", width=6,fg="black", bg="yellow", activebackground='yellow', command=peakhold)
		self.peakholdbtn.grid(row=1, column=6, padx=0, pady=0)

		#with concurrent.futures.ProcessPoolExecutor() as executor():
            #leds = executor.submit(calibrate, self, graphdata)

		#self.main_process = os.getpid()
		#graphdata = multiprocessing.Queue()
		#self.leds = multiprocessing.Process(target=self.vid.calibrate,args =[graphdata])
		self.leds = Process(target=MyVideoCapture.calibrate,args=[self.data()])
		#calibrate button
		self.calbutton = tkinter.Button(self.control_frame, text="Calibrate", width=6,
                                  fg="yellow", bg="red", activebackground='red', command=self.leds.start)
		self.calbutton.grid(row=4, column=0)



		# After it is called once, the update method will be automatically called every delay milliseconds
		if __name__ == '__main__':
			#self.vid = cv2.VideoCapture(0)#, cv2.CAP_DSHOW)

			self.delay = 15
			self.update()

			self.window.mainloop()





	def update(self):
		# Get a frame from the video source
		ret, frame = self.vid.get_frame()
		ret2, graphdata = self.vid.get_graph()
		if ret:
			self.photo = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(frame))
			self.canvas0.create_image(0, 0, image=self.photo, anchor=tkinter.NW)
		if ret2:
			self.photo2 = PIL.ImageTk.PhotoImage(image=PIL.Image.fromarray(graphdata[0]))
			self.canvas1.create_image(0, 0, image=self.photo2, anchor=tkinter.NW)
		self.window.after(self.delay, self.update)

	def data(self):
		ret2, graphdata = self.vid.get_graph()
		return graphdata




class MyVideoCapture:
	def __init__(self, calibration, video_source=0):

		self.calibration = calibration

		#settings for peak detect
		self.mindist = 50  # minumum distance between peaks
		self.thresh = 20  # Threshold
		self.savpoly = 7  # savgol filter polynomial
		self.intensity = [0] * 636  # array for intensity data...full of zeroes
		self.holdpeaks = False

		#initial graph points and wavelengths.
		self.point1 = 72  # 405nm
		self.nm1 = 405
		self.point2 = 304  # 650nm
		self.nm2 = 532

		# Open the video source
		self.vid = cv2.VideoCapture(video_source, cv2.CAP_DSHOW)
		#Settings

		self.vid.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
		self.vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
		self.vid.set(cv2.CAP_PROP_FPS, 25)

		if not self.vid.isOpened():
			raise ValueError("Unable to open video source", video_source)

		# Get video source width and height
		self.width = self.vid.get(cv2.CAP_PROP_FRAME_WIDTH)
		self.height = self.vid.get(cv2.CAP_PROP_FRAME_HEIGHT)
		#print(self.width)

	def calibrate(self,graphdata):

		def shiftR(distR, height):
			R = deque(height)  # turn list into deque
			R.rotate(distR)    # rotate deque by key
			return list(R)   # turn deque back into a list

		def shiftG(distG, height):
			G = deque(height)  # turn list into deque
			G.rotate(distG)    # rotate deque by key
			return list(G)   # turn deque back into a list

		def shiftY(distY, height):
			Y = deque(height)  # turn list into deque
			Y.rotate(distY)    # rotate deque by key
			return list(Y)
	#calibrationLabel = Label(root, text="Spectrometer Calibrated!")
	#calibrationLabel.grid(row=50, column=100)
		pointR = 618.5
		pointG = 568.5
		pointY = 587.5
		maxRed = max(graphdata[1])
		print(maxRed)
		maxG = max(graphdata[1])
		print(maxG)
		#MyVideoCapture.get_graph(self)
		#print(graphdata[1])
		calibration = ((pointR, maxRed), (pointG, maxG))
		print(calibration)
		self.recalibrate(calibration)

	def recalibrate(self, calibration):
		self.calibration = calibration


	def get_frame(self):
		if self.vid.isOpened():
			ret, frame = self.vid.read()
			if ret:
				# Return a boolean success flag and the current frame converted to BGR
				frame = cv2.resize(frame, (320, 240))  # resize the live image
				cv2.line(frame, (0, 120), (320, 120), (255, 255, 255), 1)
				return (ret, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
			else:
				return (ret, None)
		else:
			return (ret, None)

	def wavelength_to_rgb(self, nm):
		#from: https://www.codedrome.com/exploring-the-visible-spectrum-in-python/
		#returns RGB vals for a given wavelength
		gamma = 0.8
		max_intensity = 255
		factor = 0

		rgb = {"R": 0, "G": 0, "B": 0}

		if 380 <= nm <= 439:
			rgb["R"] = -(nm - 440) / (440 - 380)
			rgb["G"] = 0.0
			rgb["B"] = 1.0
		elif 440 <= nm <= 489:
			rgb["R"] = 0.0
			rgb["G"] = (nm - 440) / (490 - 440)
			rgb["B"] = 1.0
		elif 490 <= nm <= 509:
			rgb["R"] = 0.0
			rgb["G"] = 1.0
			rgb["B"] = -(nm - 510) / (510 - 490)
		elif 510 <= nm <= 579:
			rgb["R"] = (nm - 510) / (580 - 510)
			rgb["G"] = 1.0
			rgb["B"] = 0.0
		elif 580 <= nm <= 644:
			rgb["R"] = 1.0
			rgb["G"] = -(nm - 645) / (645 - 580)
			rgb["B"] = 0.0
		elif 645 <= nm <= 780:
			rgb["R"] = 1.0
			rgb["G"] = 0.0
			rgb["B"] = 0.0

		if 380 <= nm <= 419:
			factor = 0.3 + 0.7 * (nm - 380) / (420 - 380)
		elif 420 <= nm <= 700:
			factor = 1.0
		elif 701 <= nm <= 780:
			factor = 0.3 + 0.7 * (780 - nm) / (780 - 700)

		if rgb["R"] > 0:
			rgb["R"] = int(max_intensity * ((rgb["R"] * factor) ** gamma))
		else:
			rgb["R"] = 0

		if rgb["G"] > 0:
			rgb["G"] = int(max_intensity * ((rgb["G"] * factor) ** gamma))
		else:
			rgb["G"] = 0

		if rgb["B"] > 0:
			rgb["B"] = int(max_intensity * ((rgb["B"] * factor) ** gamma))
		else:
			rgb["B"] = 0

		return (rgb["R"], rgb["G"], rgb["B"])

	def get_graph(self):
		if self.vid.isOpened():
			ret, frame = self.vid.read()
			if ret:
				#Process the data...
				#Why 636 pixels? see notes on picam at beginning of file!
				piwidth = 636
				image = frame
				bwimage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
				rows, cols = bwimage.shape
				#create a blank image for the data

				graph = np.zeros([255, piwidth, 3], dtype=np.uint8)
				graph.fill(255)  # fill white

				#Display a graticule calibrated with cal data
				#calculate the ranges
				# how many px between points 1 and 2?
				pxrange = abs(self.point1-self.point2)
				# how many nm between points 1 and 2?
				nmrange = abs(self.nm1-self.nm2)
				#how many pixels per nm?
				pxpernm = pxrange/nmrange
				#how many nm per pixel?
				nmperpx = nmrange/pxrange
				#how many nm is zero on our axis
				zero = self.nm1-(self.point1/pxpernm)
				scalezero = zero  # we need this unchanged duplicate of zero for later!
				prevposition = 0
				textoffset = 12
				font = cv2.FONT_HERSHEY_SIMPLEX

				#Graticule
				#vertical lines
				for i in range(piwidth):
					position = round(zero)
					if position != prevposition:  # because of rounding, sometimes we draw twice. Lets fix tht!
						# we could have grey lines for subdivisions???S
						if position % 10 == 0:
							cv2.line(graph, (i, 15), (i, 255), (200, 200, 200), 1)
						if position % 50 == 0:
							cv2.line(graph, (i, 15), (i, 255), (0, 0, 0), 1)
							cv2.putText(graph, str(position)+'nm', (i-textoffset, 12),
							            font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)
					zero += nmperpx
					prevposition = position
				#horizontal lines
				for i in range(255):
					if i != 0 and i % 51 == 0:  # suppress the first line then draw the rest...
						cv2.line(graph, (0, i), (piwidth, i), (100, 100, 100), 1)

				#now process the data...
				halfway = int(rows/2)  # halfway point to select a row of pixels from

				#pull out  single row of data and store in a self.intensity array
				#Why -4 pixels? see notes on picam at beginning of file!
				for i in range(cols-4):
					data = bwimage[halfway, i]

					if self.holdpeaks == True:
						if data > self.intensity[i]:
							self.intensity[i] = data
					else:
						self.intensity[i] = data

				if self.holdpeaks == False:
					#do a little smoothing of the data
					self.intensity = savgol_filter(self.intensity, 17, int(self.savpoly))
				self.intensity = self.intensity.astype(int)

				#now draw the graph
				#for each index, plot a verital line derived from int
				#use waveleng_to_rgb to false color the data.

				self.wavelengthdata = []
				index = 0
				for i in self.intensity:
					wavelength = (scalezero+(index/pxpernm))
					wavelengthdata = round(wavelength, 1)
					wavelength = round(wavelength)
					self.wavelengthdata.append(wavelengthdata)
					rgb = self.wavelength_to_rgb(wavelength)
					r = rgb[0]
					g = rgb[1]
					b = rgb[2]
					#(start x,y) (end x,y) (color) thickness
					#origin is top left.
					cv2.line(graph, (index, 255), (index, 255-i), (r, g, b), 1)
					cv2.line(graph, (index, 254-i), (index, 255-i), (0, 0, 0), 1, cv2.LINE_AA)
					index += 1

				#find peaks and label them
				thresh = int(self.thresh)  # make sure the data is int.
				indexes = peakutils.indexes(
					self.intensity, thres=thresh/max(self.intensity), min_dist=self.mindist)
				for i in indexes:
					height = self.intensity[i]
					height = 245-height
					wavelength = int(scalezero+(i/pxpernm))
					cv2.rectangle(graph, ((i-textoffset)-2, height+3),
					              ((i-textoffset)+45, height-11), (255, 255, 0), -1)
					cv2.rectangle(graph, ((i-textoffset)-2, height+3),
					              ((i-textoffset)+45, height-11), (0, 0, 0), 1)
					cv2.putText(graph, str(wavelength)+'nm', (i-textoffset,
					            height), font, 0.4, (0, 0, 0), 1, cv2.LINE_AA)

				#################################################################
				graphdata = []
				graphdata.append(graph)
				graphdata.append(self.wavelengthdata)
				graphdata.append(self.intensity)

				#print("data")
				#print(graphdata)
				#print("intensity")
				#print(self.wavelengthdata)
				return (ret, graphdata)
			else:
				return (ret, None)
		else:
			return (ret, None)


	# Release the video source when the object is destroyed
	def __del__(self):
		if self.vid.isOpened():
			self.vid.release()

# Create a window and pass it to the Application object
App(tkinter.Tk(), "PySpectrometer")
#pi
