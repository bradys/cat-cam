import re, time, cv2, serial


'''
ServoController interfaces with the arduino board to control the servo motor over 
USB serial coms
'''


class ServoController:

    def __init__(self):
        self.ser = serial.Serial('com3', 9600, timeout=0.5)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.ser.close()

    def servo_read_position(self):
        self.ser.write(b'r')
        pos = str(self.ser.readline())
        # First read is always empty, sometimes the first severl,
        # so recusively call until numbers come back from arduino
        while pos == "b''":
            pos = self.servo_read_position()
        print("pos_remote_read: " + str(pos))
        m = re.search('\d+', str(pos))
        pos = int(m.group(0))
        return pos

    def servo_set_position(self, pos):
        print("ad pos: " +str(pos))
        self.ser.write(bytes(str(pos), 'utf8'))
        self.ser.write(b'\n')
        print("pos_remote: " + str(self.ser.readline()))


'''
SerialServo controls camera rotation and motion detection
Currently written to sweep the camera, wait and detect motion, only continue sweeping once no motion detected
'''


class SerialServo:

    def camera_frame_grab(self, frameName, capture):
        ret, frame = capture.read()
        cv2.imshow(frameName, frame)
        cv2.waitKey(1)

    def camera_rotate(self, start, stop, capture, step_delay=0.2):
        # If the stop angle bigger than start, increment, else decrement
        # If the stop angle is smaller, it cannot be negative, if it's bigger,
        # then it can't be bigger than 180
        if start <= stop:
            if stop < 25:
                stop = 25
            direction = 1
        else:
            if stop > 165:
                stop = 165
            direction = -1

        for pos in range(start, stop, direction):
            self.camera_frame_grab('frame', capture)
            print("af pos: " + str(pos))
            sc.servo_set_position(pos)
            time.sleep(step_delay)

    def camera_sweep(self):
        self.camera_rotate(45, 180)
        self.camera_rotate(180, 45)

    def camera_left(self, degrees, capture):
        pos = sc.servo_read_position()
        if pos <= 180:
            self.camera_rotate(pos, pos + degrees)

    def camera_right(self, degrees, capture):
        pos = sc.servo_read_position()
        if pos > 0:
            self.camera_rotate(pos, pos - degrees, capture)

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

            ret, thresh = cv2.threshold(fgmask, 127, 255, 0)
            thresh = cv2.dilate(thresh, None, iterations=2)
            im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            count = 1
            # loop over the contours
            for c in contours:
                if cv2.contourArea(c) < 500:
                    continue
                count += count
                # (x, y, w, h) = cv2.boundingRect(c)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.imshow('frame', frame)
            k = cv2.waitKey(60) & 0xff
            if k == 27:
                break

    def __init__(self):

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

if __name__ == "__main__":
    with ServoController() as sc:
        with SerialServo() as ss:
            while(True):
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_right(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
                ss.camera_left(25, ss.cap)
                ss.detectMotion(20)
