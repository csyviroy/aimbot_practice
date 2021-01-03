import pyautogui as g
import numpy as np
import cv2
import math
from PIL import ImageGrab
import time


class Monitor:
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self.screen_x_length = g.size().width
		self.screen_y_length = g.size().height

	@classmethod
	def get_screen_frame(self):
		open_cv_image = np.array(ImageGrab.grab())
		# Convert RGB to BGR
		return open_cv_image[:, :, ::-1].copy()


class Trainer3DSqaure:
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	@classmethod
	def read_object_position(self, frame):
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blur = cv2.medianBlur(gray, 5)
		sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
		sharpen = cv2.filter2D(blur, -1, sharpen_kernel)
		ret, thresh = cv2.threshold(sharpen, 170, 255, cv2.THRESH_BINARY)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
		close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)
		close = cv2.bitwise_not(close)

		close = cv2.dilate(close, kernel, iterations=8)

		cnts = cv2.findContours(close, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		cnts = cnts[0] if len(cnts) == 2 else cnts[1]
		min_area = 9000
		max_area = 13000
		image_number = 0

		objects = []
		for c in cnts:
			area = cv2.contourArea(c)
			if area > min_area and area < max_area:
				x, y, w, h = cv2.boundingRect(c)

				objects.append([math.ceil(x + (w / 2)), math.ceil(y + (h / 2))])

		if len(objects) == 0:
			return None
		return objects


class MouseAccuracy:
	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

	@classmethod
	def read_object_position(self, frame):
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		y = 220
		x = 0
		w = 1828
		h = 810

		mask = np.zeros(gray.shape, np.uint8)
		mask[y:y + h, x:x + w] = gray[y:y + h, x:x + w]
		rows = mask.shape[0]

		circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=100, param2=30, minRadius=1, maxRadius=30)

		objects = []

		if circles is not None:
			circles = np.uint16(np.around(circles))
			for i in circles[0, :]:
				center = (i[0], i[1])
				objects.append(center)

		if len(objects) == 0:
			return None
		return objects


def config_bot(method, *args, **kwargs):
	if method == 1:
		base = MouseAccuracy
	else:
		base = Trainer3DSqaure

	class AimBot(Monitor, base):
		def __init__(self, fx=None, fy=None, *args, **kwargs):
			super().__init__(*args, **kwargs)
			if fx is not None:
				self.compensate_factor_x = fx
				if fy is not None:
					self.compensate_factor_y = fy
				else:
					self.compensate_factor_y = fx
			else:
				self.compensate_factor_x = 1
				self.compensate_factor_y = 1

		@classmethod
		def get_mouse_position(self):
			return g.position()

		@classmethod
		def calibrate_mouse_position(self):
			g.moveTo(self.screen_x_length / 2, self.screen_y_length / 2, duration=0)

		@classmethod
		def populate_best_relative_dist(self, objects):
			mouse_position = self.get_mouse_position()
			print('current mouse position: ', mouse_position)
			closest = min(objects, key=lambda obj: abs(obj[0] - mouse_position.x) + abs(obj[1] - mouse_position.y))
			print('closest: ', closest)
			return (closest[0] - mouse_position.x, closest[1] - mouse_position.y)

		def emit_mouse_action(self, rel):
			print('travel:', self.compensate_factor_x * rel[0], ', ', self.compensate_factor_y * rel[1])
			g.moveRel(self.compensate_factor_x * rel[0], self.compensate_factor_y * rel[1], duration=0)
			print('new mouse position: ', self.get_mouse_position())
			g.click()

	return AimBot(**kwargs)


def main():
	ab = config_bot(method=1, fx=1)
	#check class creation flow
	print(ab.__class__.__mro__)
	try:
		while True:
			frame = ab.get_screen_frame()
			objs = ab.read_object_position(frame)
			if objs is not None:
				print('clicking')
				cord_diff = ab.populate_best_relative_dist(objs)
				ab.emit_mouse_action(cord_diff)
	except KeyboardInterrupt:
		pass

if __name__ == "__main__":
	print('Ready')
	time.sleep(5)
	print('start')
	main()
	print('complete iteration')
