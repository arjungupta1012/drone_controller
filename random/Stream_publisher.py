import cv2
import threading
import paho.mqtt.client as mqtt


class Stream_publisher:
    
    def __init__(self, topic, video_address=0, start_stream=True, host="3.110.177.25", port=1883, socket_timeout=60000):
        
        self.client = mqtt.Client()
        try:
            self.client.connect(host, port)
        except Exception as e:
            print(f"Error connecting to MQTT broker: {e}")
        
        # Set socket timeout for the MQTT client
        try:
            self.client.socket().settimeout(socket_timeout)
        except Exception as e:
            print(f"Error setting socket timeout: {e}")
        
        self.topic = topic
        self.video_source = video_address

        self.cam = cv2.VideoCapture(self.video_source)
        #width  = self.cam.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH)   # float `width`
        #height = self.cam.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)  # float `height`
        # or
        width  = self.cam.get(3)  # float `width`
        height = self.cam.get(4)  # float `height`

        # it gives me 0.0 :/
        #fps = self.cam.get(cv2.cv.CV_CAP_PROP_FPS)
        print(width)
        print(height)
        #print(fps)
        self.streaming_thread = threading.Thread(target=self.stream)
        if start_stream:
            self.streaming_thread.start()
    
    def start_streaming(self):
        self.streaming_thread.start()

    def stream(self):
        print("Streaming from video source: {}".format(self.video_source))
        while True:
            _, img = self.cam.read()
            
            if img is None:
                break  # Exit the loop if there are no more frames

            img_str = cv2.imencode('.jpg', img)[1].tobytes()
            self.client.publish(self.topic, img_str)
            

if __name__ == "__main__":
    # Example usage:
    webcam = Stream_publisher("test", video_address=0, socket_timeout=60000)

    
    # ...

