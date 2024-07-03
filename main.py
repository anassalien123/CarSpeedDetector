import cv2
import dlib
import time
import math

# Load the cascade classifier for detecting cars
carCascade = cv2.CascadeClassifier('file.xml')

# Open the video file
video = cv2.VideoCapture('traffic.mp4')

# Define the width and height for resizing the video frames
WIDTH = 1280
HEIGHT = 720


def estimateSpeed(location1, location2):
    # Calculate the Euclidean distance between two points
    d_pixels = math.sqrt(math.pow(location2[0] - location1[0], 2) + math.pow(location2[1] - location1[1], 2))

    # Assuming 9.25 pixels per meter ratio (you may need to adjust this based on your setup)
    ppm = 14.25

    # Convert pixel distance to meters
    d_meters = d_pixels / ppm

    # Assuming FPS (frames per second) is 30
    fps = 60

    # Calculate speed in km/h
    speed = d_meters * fps * 3.6

    return speed


def trackMultipleObjects():
    # Colors for drawing rectangles on tracked objects
    rectangleColor1 = (0, 0, 255)  # Red
    rectangleColor2 = (0, 255, 0)  # Green

    frameCounter = 0
    currentcarID = 0
    fps = 0

    carTracker = {}
    carLocation1 = {}
    carLocation2 = {}
    speed = [None] * 1000

    while True:
        start_time = time.time()
        rc, image = video.read()

        if type(image) == type(None):
            break

        image = cv2.resize(image, (WIDTH, HEIGHT))
        resultImage = image.copy()

        frameCounter = frameCounter + 1

        carIDtoDeletes = []

        for carID in carTracker.keys():
            trackingQuality = carTracker[carID].update(image)

            if trackingQuality < 7:
                carIDtoDeletes.append(carID)

        for carID in carIDtoDeletes:
            print('Removing carID ' + str(carID) + ' from list of trackers.')
            print('Removing carID ' + str(carID) + ' previous location.')
            print('Removing carID ' + str(carID) + ' current tracker.')

            carTracker.pop(carID, None)
            carLocation1.pop(carID, None)
            carLocation2.pop(carID, None)

        if not (frameCounter % 10):
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            cars = carCascade.detectMultiScale(gray, 1.1, 13, 18, (24, 24))

            for (_x, _y, _w, _h) in cars:
                x = int(_x)
                y = int(_y)
                w = int(_w)
                h = int(_h)

                x_bar = x + 0.5 * w
                y_bar = y + 0.5 * h

                cv2.rectangle(resultImage, (x, y), (x + w, y + h), rectangleColor1, 4)

                matchCarID = None

                for carID in carTracker.keys():
                    trackedPosition = carTracker[carID].get_position()

                    t_x = int(trackedPosition.left())
                    t_y = int(trackedPosition.top())
                    t_w = int(trackedPosition.width())
                    t_h = int(trackedPosition.height())

                    t_x_bar = t_x + 0.5 * t_w
                    t_y_bar = t_y + 0.5 * t_h

                    if ((t_x <= x_bar <= (t_x + t_w)) and (t_y <= y_bar <= (t_y + t_h)) and (
                            x <= t_x_bar <= (x + w)) and (y <= t_y_bar <= (y + h))):
                        matchCarID = carID

                if matchCarID is None:
                    print('Creating new tracker ' + str(currentcarID))

                    tracker = dlib.correlation_tracker()
                    tracker.start_track(image, dlib.rectangle(x, y, x + w, y + h))

                    carTracker[currentcarID] = tracker
                    carLocation1[currentcarID] = [x, y, w, h]

                    currentcarID = currentcarID + 1

            for carID in carTracker.keys():
                trackedPosition = carTracker[carID].get_position()

                t_x = int(trackedPosition.left())
                t_y = int(trackedPosition.top())
                t_w = int(trackedPosition.width())
                t_h = int(trackedPosition.height())

                cv2.rectangle(resultImage, (t_x, t_y), (t_x + t_w, t_y + t_h), rectangleColor2, 4)

                carLocation2[carID] = [t_x, t_y, t_w, t_h]

        end_time = time.time()

        if not (end_time == start_time):
            fps = 1.0 / (end_time - start_time)

        for i in carLocation1.keys():
            if frameCounter % 1 == 0:
                [x1, y1, w1, h1] = carLocation1[i]
                [x2, y2, w2, h2] = carLocation2[i]

                carLocation1[i] = [x2, y2, w2, h2]

                if [x1, y1, w1, h1] != [x2, y2, w2, h2]:
                    if (speed[i] == None or speed[i] == 0) and y1 >= 100 and y1 <= 700:
                        speed[i] = estimateSpeed([x1, y1, w1, h1], [x2, y2, w2, h2])

                        if speed[i] != None and y1 >= 180:
                            cv2.putText(resultImage, str(int(speed[i])) + ' km/hr', (int(x1 + w1 / 2), int(y1 - 5)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)

                cv2.imshow('Result', resultImage)

                if cv2.waitKey(33) == 27:
                    break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    trackMultipleObjects()













