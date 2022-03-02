import numpy as np
import imutils
import cv2
from statistics import mean

data = {
	"green": { "left": '1', "right": '0' },
	"red": { "up": '2', "down": '3', "semi": '8' },
	"light_green": '4',
	"yellow": '5',
	"pink": '6',
	"light_blue": '7',
	"blue": '9',
	"brown": { "triangle": '.', "square": '-', "circle": ',' }
}

green_boundary = ([0, 200, 0], [50, 255, 50])
red_boundary = ([0, 0, 150], [10, 10, 255])
light_green_boundary = ([50, 160, 100], [150, 200, 120])
yellow_boundary = ([0, 200, 200], [100, 255, 255])
pink_boundary = ([100, 150, 200], [200, 200, 255])
light_blue_boundary = ([200, 150, 150], [255, 200, 180])
blue_boundary = ([100, 50, 0], [200, 100, 50])
brown_boundary = ([0, 0, 100], [20, 50, 200])

class LocationDetector:
	def __init__(self):
		# self.image = image
		self.threshold = 50
		pass


	def detect_location(self, image):
		coordinates = self.detect_coordinates(image)
		coordinates = self.filter_coordinates(coordinates)

		coordi = ""
		for c in coordinates:
			coordi += c[1]

		lat, lon = map(lambda x: float(x), coordi.split(","))

		return lat, lon


	def get_countours(self, image, boundary):
		(lower, upper) = boundary
		lower = np.array(lower, dtype = "uint8")
		upper = np.array(upper, dtype = "uint8")
		mask = cv2.inRange(image, lower, upper)
		output = cv2.bitwise_and(image, image, mask = mask)
		output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(output, (0,0), sigmaX=33, sigmaY=33)
		output = cv2.divide(output, blur, scale=255)
		output = cv2.threshold(output, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
		output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel)
		contours, _ = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contours


	def get_x(self, approx):
		approx = sorted([ a[0][0] for a in approx ])
		return int(mean(approx))


	def detect_coordinates(self, image):
		coordinates = []

		# green
		contours = self.get_countours(image, green_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			b = sorted([ a[0][0] for a in approx ])
			if b[2] - b[0] > b[-1] - b[2]:
				coordinates.append([self.get_x(approx), data["green"]["right"]])
			else :
				coordinates.append([self.get_x(approx), data["green"]["left"]])

		# red
		contours = self.get_countours(image, red_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			if len(approx) < 10:
				b = sorted([ a[0][1] for a in approx ])
				if b[2] - b[0] > b[-1] - b[2]:
					coordinates.append([self.get_x(approx), data["red"]["down"]])
				else :
					coordinates.append([self.get_x(approx), data["red"]["up"]])
			else:
				coordinates.append([self.get_x(approx), data["red"]["semi"]])

		# light_green
		contours = self.get_countours(image, light_green_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			coordinates.append([self.get_x(approx), data["light_green"]])

		# yellow
		contours = self.get_countours(image, yellow_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			coordinates.append([self.get_x(approx), data["yellow"]])

		# pink
		contours = self.get_countours(image, pink_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			coordinates.append([self.get_x(approx), data["pink"]])

		# light_blue
		contours = self.get_countours(image, light_blue_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			coordinates.append([self.get_x(approx), data["light_blue"]])

		# blue
		contours = self.get_countours(image, blue_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			coordinates.append([self.get_x(approx), data["blue"]])

		# brown
		contours = self.get_countours(image, brown_boundary)
		for contour in contours:
			approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True).tolist()
			if len(approx) == 3:
				coordinates.append([self.get_x(approx), data["brown"]["triangle"]])
			elif len(approx) == 4:
				coordinates.append([self.get_x(approx), data["brown"]["square"]])
			else:
				coordinates.append([self.get_x(approx), data["brown"]["circle"]])

		coordinates = sorted(coordinates, key=lambda x: x[0])

		return coordinates


	def filter_coordinates(self, coordinates):
		idx = 1
		while idx != len(coordinates):
			if coordinates[idx][1] == coordinates[idx-1][1] and abs(coordinates[idx][0] - coordinates[idx-1][0]) < self.threshold:
				coordinates.pop(idx-1)
				continue
			idx += 1

		return coordinates