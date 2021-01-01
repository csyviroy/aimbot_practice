import pyautogui as g
import numpy as np
import cv2
from PIL import ImageGrab
import time

class Monitor:
	def __init(self):
		self.screen_x_length = g.size().width
		self.screen_y_length = g.size().height

	def get_screen_frame(self):
		open_cv_image = numpy.array(ImageGrab.grab()) 
		# Convert RGB to BGR 
		return open_cv_image[:, :, ::-1].copy()

class ObjectDetector:
	def read_object_position(self, frame):
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		blur = cv2.medianBlur(gray, 5)
		sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
		sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
		ret, thresh = cv2.threshold(sharpen,170,255, cv2.THRESH_BINARY)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
		close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)
		close = cv2.bitwise_not(close)
	
		close = cv2.dilate(close,kernel,iterations = 8)

		cnts = cv2.findContours(close, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		cnts = cnts[0] if len(cnts) == 2 else cnts[1]
		min_area = 9000
		max_area = 13000
		image_number = 0

		objects = []
		for c in cnts:
			area = cv2.contourArea(c)
			if area > min_area and area < max_area:
				#x,y,w,h = cv2.boundingRect(c)
				#ROI = image[y:y+h, x:x+h]
				#cv2.rectangle(image, (x, y), (x + w, y + h), (36,255,12), 2)
				#image_number += 1
				objects.append([x+(w/2), y+(h/2)])
		return objects

class AimBot(Monitor, ObjectDetector):
	def __int__(self):
		super(Monitor, self).__init__()

	def get_mouse_position(self):
		return g.position()

	def populate_relative_dist(object):
		obj_x = object[0]
		obj_y = object[1]
		mouse_position = self.get_mouse_position()
		return mouse_position.x-obj_x, mouse_position.y-obj_y

	def emit_mouse_action(self, rel_x, rel_y):
		g.moveRel(rel_x, rel_y, duration=0.5)
		g.click()

def main():
	ab = AimBot()
	frame = ab.get_screen_frame()
	objs = ab.read_object_position(frame)
	for obj in objs:
		print('clicking')
		cord_diff = ab.populate_relative_dist(obj)
		ab.emit_mouse_action(cord_diff[0], cord_diff[1])

if __name__ == "__main__":
	time.sleep(5)
	main()
	print('complete iteration')