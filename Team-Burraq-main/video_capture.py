import cv2
import threading

class VideoDisplay(threading.Thread):
    def __init__(self, name, frame):
        super().__init__(name=name)
        self.frame = frame
        self.stopped = False

    def run(self):
        while not self.stopped:
            cv2.imshow(self.getName(), self.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop()

    def stop(self):
        self.stopped = True
        cv2.destroyAllWindows()
