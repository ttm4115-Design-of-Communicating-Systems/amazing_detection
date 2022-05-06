import json
from threading import Thread

import cv2
import paho.mqtt.client as mqtt
from stmpy import Driver, Machine
import time


class OfficeRPIMovement:
    def __init__(self, rpi_id):
        self.id = rpi_id

    def publish(self):
        print("Published ID")
        self.mqtt_client.publish("ABS/office/movement", self.id)

    def test(self):
        print("test")


t0 = {"source": "initial", "target": "idle"}

t1 = {
    "trigger": "movement_detected",
    "source": "idle",
    "target": "movement",
    "effect": "start_timer('t', 10000); publish()",
}

t2 = {"trigger": "movement_detected", "source": "movement", "target": "movement"}

t3 = {"trigger": "t", "source": "movement", "target": "idle"}


class MQTT_Client:
    def __init__(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect

    def on_connect(self, client, userdata, flags, rc):
        print("on_connect(): {}".format(mqtt.connack_string(rc)))

    def start(self, broker, port):

        print("Connecting to {}:{}".format(broker, port))
        self.client.connect(broker, port)

        try:
            thread = Thread(target=self.client.loop_forever)
            thread.start()
        except KeyboardInterrupt:
            print("Interrupted")
            self.client.disconnect()


class MovementDetection:

    def detectMovement():
        previous_frame = None

        video = cv2.VideoCapture(0)

        while True:
            time.sleep(0.1)
            ret, frame = video.read()

            motion = 0

            current_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_frame = cv2.GaussianBlur(current_frame, (21, 21), 0)

            if previous_frame is None:
                previous_frame = current_frame

            diff_frame = cv2.absdiff(previous_frame, current_frame)

            thresh_frame = cv2.threshold(diff_frame, 50, 255, cv2.THRESH_BINARY)[1]
            thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)

            contours, _ = cv2.findContours(
                thresh_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            for contour in contours:
                if cv2.contourArea(contour) < 7500:
                    motion = 1

            if motion == 1:
                print("Movement detected!")
                myclient.stm_driver.send(
                    "movement_detected", "movement", args=None, kwargs=None
                )

            previous_frame = current_frame

            cv2.imshow("Difference Frame", diff_frame)

            key = cv2.waitKey(1)
            if key == ord("q"):
                break

        video.release()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    data = {}
    with open("./id.json") as f:
        data = json.load(f)

    broker, port = "localhost", 1883

    m = OfficeRPIMovement(rpi_id=data["id"])
    m_machine = Machine(transitions=[t0, t1, t2, t3], obj=m, name="movement")
    m.stm = m_machine

    driver = Driver()
    driver.add_machine(m_machine)

    myclient = MQTT_Client()
    m.mqtt_client = myclient.client
    myclient.stm_driver = driver

    driver.start()
    myclient.start(broker, port)

    MovementDetection.detectMovement()
