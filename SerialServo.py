import re, time, cv2, imutils, serial
import numpy as np

'''
SerialServo controls camera rotation and motion detection
Currently written to sweep the camera, wait and detect motion, only continue sweeping once no motion detected
'''
class SerialServo:

    def rotate_camera(self, start, stop, step_delay=0.2):
        # If the stop angle bigger than start, increment, else decrement
        # If the stop angle is smaller, it cannot be negative, if it's bigger,
        # then it can't be bigger than 180

        print("Started motion detect")
        if start <= stop:
            if stop < 25:
                stop = 25
            for pos in range(start, stop):

                ret, frame = self.cap.read()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

                self.ser.write(bytes(str(pos), 'utf8'))
                self.ser.write(b'\n')
                print("pos_remote: " + str(self.ser.readline()))
                time.sleep(step_delay)
        else:
            if stop > 165:
                stop = 165
            for pos in range(start, stop, -1):

                ret, frame = self.cap.read()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)

                self.ser.write(bytes(str(pos), 'utf8'))
                self.ser.write(b'\n')
                print("pos_remote: " + str(self.ser.readline()))
                time.sleep(step_delay)
        print("Stopped motion detect")

    def camera_sweep(self):
        print("45 to 90\n")
        self.rotate_camera(45, 90)
        time.sleep(5)
        print("90 to 180\n")
        self.rotate_camera(90, 180)
        time.sleep(5)
        print("180 to 90\n")
        self.rotate_camera(180, 90)
        time.sleep(5)
        print("90 to 45\n")
        self.rotate_camera(90, 45)
        time.sleep(5)

    def camera_left(self, degrees):
        pos = str(self.ser.readline())
        print("pos_remote_read: " + pos)
        while pos == "b''":
            self.ser.write(b'r')
            pos = str(self.ser.readline())
            print("pos_remote_read: " + pos)

        m = re.search('\d+', pos)
        if int(m.group(0)) <= 180:
            self.rotate_camera(int(m.group(0)), int(m.group(0))+degrees)

    def camera_right(self, degrees):
        pos = str(self.ser.readline())
        print("pos_remote_read: " + pos)
        while pos == "b''":
            self.ser.write(b'r')
            pos = str(self.ser.readline())
            print("pos_remote_read: " + pos)

        m = re.search('\d+', pos)
        if int(m.group(0)) > 0:
            self.rotate_camera(int(m.group(0)), int(m.group(0))-degrees)

    def detectMotion(self, duration):
        start_time = time.time()
        while (True):
            end_time = time.time()
            if(end_time - start_time) > duration and count == 1:
                print("Finished recording " + str(duration))
                break
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            fgmask = self.fgbg.apply(frame)

            # ret, thresh = cv2.threshold(fgmask, 127, 255, 0)
            imgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(fgmask, 127, 255, 0)

            thresh = cv2.dilate(thresh, None, iterations=2)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            count = 1
            # loop over the contours
            for c in contours:
                if cv2.contourArea(c) < 500:
                    continue
                count+=count
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # print("Number of contours: " + str(count))
            cv2.imshow('frame', frame)
            k = cv2.waitKey(60) & 0xff
            if k == 27:
                break

    def __init__(self):
        self.ser = serial.Serial('com3', 9600, timeout=0.5)

        self.cap = cv2.VideoCapture(1)
        self.fgbg = cv2.createBackgroundSubtractorMOG2()
        ret, frame = self.cap.read()
        cv2.imshow('frame', frame)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        # When everything done, release the capture and serial port
        self.cap.release()
        cv2.destroyAllWindows()
        self.ser.close()

if __name__ == "__main__":
    with SerialServo() as ss:
        while(True):
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_right(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
            ss.camera_left(25)
            ss.detectMotion(20)
