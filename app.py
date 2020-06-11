#!/usr/bin/env python
# -*-encoding:utf-8-*-
import json
from queue import Queue, Empty

# import rosnode
# import rospy
from enum import IntEnum, Enum

from common import Config
from common import Timer, EventType, Event
from connector import Connector
from main import  Carla_Client
from drivingtask import DrivingTask
import time
# from web_server import WebServer

class ActionType:
    REPLAY = "REPLAY"
    LANE = "LANE"
    TRAJECTORY = "TRJ"
    SITE = "SITE"
    LIDAR = "LIDAR"

class ProtocolFactory:
    def __init__(self, config):
        self.config = config
        self.request_id = -1

    def encoder_response(self, cmd, request_id, ack, data=None):
        msg = dict()
        msg['version'] = self.config.version
        msg['type'] = int(cmd)
        msg['ack'] = int(ack)
        msg['requestId'] = request_id
        msg['vin'] = self.config.vin
        if data:
            msg['data'] = json.dumps(data)
        return json.dumps(msg)

    def _get_next_request_id(self):
        self.request_id = self.request_id + 1
        return self.request_id

    def encoder_request(self, cmd, data=None):
        msg = dict()
        msg['version'] = self.config.version
        msg['type'] = int(cmd)
        msg['ack'] = int(ACK.COMMAND)
        msg['requestId'] = self._get_next_request_id()
        msg['vin'] = self.config.vin
        if data:
            msg['data'] = json.dumps(data)
            print(msg['data'])


        return json.dumps(msg)


class TerminalHealth:
    def __init__(self):
        self.gps = -1
        self.camera = -1
        self.can = -1

    def is_all_received(self):
        return self.gps != -1 and self.camera != -1 and self.can != -1


class ACK(IntEnum):
    COMMAND = 0xfe
    SUCCESS = 0x00
    NOT_AUTHORIZED = 0x01
    FAILED = 0x02
    ILLEGAL_TOKEN = 0x03
    ILLEGAL_PROTOCOL = 0x04
    REPEATED = 0x05
    NOT_SUPPORTED_PROTOCOL = 0x06
    SEVER_ERROR = 0x07


class Command:
    # 协议中type类型值
    REGISTRATION = 0x00
    LOGIN = 0x01
    HEARTBEAT = 0x02
    LOGOUT = 0x03
    MODE_CHANGE = 0x04
    MODULE_CHECK = 0x05
    TASK_REQUEST = 0x10
    TASK_REAL_TIME_INFO = 0x011
    GPS_INFO = 0x12
    STOP_AUTONOMOUS = 0x13
    OBSTACLE_INFO = 0x14
    REALTIME_CAR_INFO = 0x15
    PREPARE_AUTONOMOUS = 0x20
    FIRE_AUTONOMOUS = 0x21
    LANE_AUTONOMOUS = 0x22
    TELE_CONTROL = 0x23
    TELE_REPORT = 0x24
    PAUSE_AUTONOMOUS = 0x31
    CONTINUE_AUTONOMOUS = 0x32
    REQUEST_LOCK = 0x41
    RESPONSE_LOCK = 0x42
    RELEASE_LOCK = 0x43


class App:
    def __init__(self):
        # rospy.init_node('Gateway', log_level=rospy.DEBUG)
        # server_config_file = rospy.get_param("~server_config_file")
        self.taskId = 0
        self.config = Config("config/server_config.ini")
        self.pf = ProtocolFactory(self.config)
        self.run_id = "123"
        # print("runid = ", self.run_id)
        self.node_list = ['123','123']
        self.timer = Timer()
        # self.monitor = Monitor(self.node_list, self.timer)
        self._server_request = {}  # stores server_request
        self._event_bus = Queue()
        self._heartbeat_timeout_job = None
        self._tele_report_job = None
        self._report_car_job = None
        self._report_task_job = None
        self.__client = Connector(self._event_bus, self.config.host,
                                  self.config.port)
        self._handler_map = {}
        self._event_handler_map = {}
        self._add_command_handler()
        self._add_event_handler()
        self.monitor = None
        # self._web_server = WebServer(self.monitor, self._service, self.config)

    def _register(self, command, func):
        """
        :type command int
        :type func function
        :return:
        """
        self._handler_map[command] = func

    # 车端和云端协议一致
    # 定义接收事件处理
    def _add_command_handler(self):
        self._register(Command.LOGIN, self.on_login)
        self._register(Command.HEARTBEAT, self.on_heartbeat)
        self._register(Command.MODE_CHANGE, self.on_mode_change)
        self._register(Command.MODULE_CHECK, self.on_module_check)
        self._register(Command.STOP_AUTONOMOUS, self.stop_autonomous)
        self._register(Command.PREPARE_AUTONOMOUS, self.prepare_autonomous)
        self._register(Command.LANE_AUTONOMOUS, self.fire_lane_autonomous)
        # self._register(Command.TELE_CONTROL, self.send_control_command_to_can)
        self._register(Command.PAUSE_AUTONOMOUS, self.pause_autonomous)
        self._register(Command.CONTINUE_AUTONOMOUS, self.continue_autonomous)
        self._register(Command.RESPONSE_LOCK, self.response_lock)
        self._register(Command.TELE_CONTROL, self.tele_control)

    def _register_event_handler(self, eventType, func):
        self._event_handler_map[eventType] = func

    # 定义车端事件处理
    def _add_event_handler(self):
        self._register_event_handler(EventType.ConnectionMadeEvent, self.on_connection_made)
        self._register_event_handler(EventType.DataReceivedEvent, self.on_data_received)
        self._register_event_handler(EventType.ConnectionDisconnectedEvent, self.on_disconnected)
        self._register_event_handler(EventType.TaskFinishedEvent, self.on_task_finished)
        self._register_event_handler(EventType.Request_CrossLockEvent, self.on_request_crosslock)
        self._register_event_handler(EventType.Release_CrossLockEvent, self.on_release_crosslock)
        self._register_event_handler(EventType.Vehicle_Update,self.on_vehicle_update)

    def dispatch(self, msg):
        message = json.loads(msg)
        cmd = int(message["type"])
        #通过_handler_map来进行处理
        func = self._handler_map.get(cmd)
        if func:
            func(message)

    def on_vehicle_update(self):
        self.client.update()

    def tele_control(self, message):
        self._tele_control_service.handle_message(message)

    def _send_login(self):
        # rospy.loginfo("[TaskIO] send registration")
        print("[TaskIO] send registration")
        data = dict()
        data['runId'] = self.run_id
        data['nodeList'] = self.node_list
        self.__client.write(self.pf.encoder_request(Command.LOGIN, data=data))

    def on_login(self, message):
        ack = int(message["ack"])
        if ack == ACK.SUCCESS:
            # rospy.loginfo("[TaskIO] login success")
            print("[TaskIO] login success")

            # 该处定义的定时任务包括 1.发送心跳消息 2. 发送车辆状态 3. 发送车辆位置
            print("[TaskIO] heartbeat")
            self._heartbeat_timeout_job = "heartbeat_job"
            self.timer.submit_periodical_job(self._send_heartbeat, 60, self._heartbeat_timeout_job)

            self._report_car_job = "report_car_job"
            self.timer.submit_periodical_job(
                self._report_car_status, 1, self._report_car_job)

            self._update_virtual_car = "update_virtual_car"
            self.timer.submit_periodical_job(
                self.on_vehicle_update, 0.1, self._update_virtual_car)
        else:
            print("[app] login to server failed")
            # rospy.loginfo("[app] login to server failed")

    def _send_tele_report(self):
        data = self.pf.encoder_request(Command.TELE_REPORT,self._tele_control_service.car_info())
        # rospy.loginfo("send_tele_report {}".format(data))
        self.__client.write(data)

    def _send_heartbeat(self):
        # rospy.loginfo("[app] send heartbeat")
        print("[app] send heartbeat")
        self.__client.write(self.pf.encoder_request(Command.HEARTBEAT))

    def on_heartbeat(self, msg):
        # rospy.loginfo("[app] received heartbeat response")
        print("[app] received heartbeat response")

    def _report_car_status(self, ):
        # msg = self.monitor.get_car_status()
        msg = None
        msg = self.client.get_car_status()
        print("[app] report car status")
        print(msg)
        data = self.pf.encoder_request(Command.REALTIME_CAR_INFO, msg)
        self.__client.write(data)

    def on_mode_change(self, message):
        # rospy.loginfo("[TaskIO] handle mode change %s " % message)
        print("[TaskIO] handle mode change %s " % message)
        self.__client.write(
            self.pf.encoder_response(Command.MODE_CHANGE, message['requestId'], ACK.SUCCESS))

    def on_module_check(self, message):
        # response = self.monitor.get_car_status()
        response = None
        data = self.pf.encoder_response(Command.MODULE_CHECK, int(
            message['requestId']), ACK.SUCCESS, data=response)
        self.__client.write(data)

    def prepare_autonomous(self, message):
        # rospy.loginfo("received prepare request")
        print("received prepare request")
        request_id = int(message['requestId'])
        if self._service.is_free():
            data = self.pf.encoder_response(Command.PREPARE_AUTONOMOUS,
                                            request_id, ACK.SUCCESS)
        else:
            data = self.pf.encoder_response(Command.PREPARE_AUTONOMOUS,
                                            request_id, ACK.FAILED)
        self.__client.write(data)

    def send_control_command_to_can(self, message):
        data = json.loads(message['data'])
        request_id = int(message['requestId'])
        self._tele_control_service.tele_control(data)
        #
        # data = self.pf.encoder_response(Command.TELE_CONTROL, request_id, ACK.SUCCESS)
        # self.__client.write(data)

    # TODO: 完善这个代码 接收到路径的路径之后的反应
    def fire_lane_autonomous(self, message):
        data = json.loads(message['data'])
        response = self.pf.encoder_response(
        Command.LANE_AUTONOMOUS, int(message['requestId']), ACK.SUCCESS)
        # print(message)
        if data:
            # taskId = data['route']['taskId']
            taskId = 1
            speed = float(data['speed'])
            driving_task = DrivingTask(taskId, speed, data['route'])
            for each in driving_task.sections[0].waypoints:
                print(each.x,each.y)
                self.client.cx.append(each.x)
                self.client.cy.append(each.y)
            # 还有task_id speed 等等
            # 去发布消息 sections = [] 里面是 TrjSectionTask
            # if self._service.start_driving_task(driving_task):
            #     response = self.pf.encoder_response(
            #         Command.LANE_AUTONOMOUS, int(message['requestId']), ACK.SUCCESS)
            #     self._report_task_job = taskId
            #     self.timer.submit_periodical_job(
            #         self._report_task, 5, self._report_task_job)
            # else:
            #     print("4- app.py fire_lane_autonomous: start_driving_task failed")
            #     response = self.pf.encoder_response(
            #         Command.LANE_AUTONOMOUS, int(message['requestId']), ACK.FAILED)
        self.__client.write(response)

    def _report_task(self):
        # rospy.loginfo("[app]report_task...")
        print("[app]report_task...")
        data = self._service.get_task_status(self._report_task_job)
        response = self.pf.encoder_request(Command.TASK_REAL_TIME_INFO, data)
        self.__client.write(response)

    def continue_autonomous(self, message):
        # rospy.loginfo("[TaskIO] continue autonomous %s" % message)
        print("[TaskIO] continue autonomous %s" % message)
        response = self.pf.encoder_response(
            Command.CONTINUE_AUTONOMOUS, message['requestId'], ACK.SUCCESS)
        self.__client.write(response)
        # self._service.task_continue()

    def response_lock(self, message):
        # rospy.loginfo("[TaskIO] Response Lock %s" % message)
        print("[TaskIO] Response Lock %s" % message)
        data = json.loads(message['data'])
        response = self.pf.encoder_response(
            Command.RESPONSE_LOCK, message['requestId'], ACK.SUCCESS)
        self.__client.write(response)
        # self._service.response_lock(data)

    def pause_autonomous(self, message):
        # rospy.loginfo("[TaskIO] pause autonomous %s" % message)
        print("[TaskIO] pause autonomous %s" % message)
        response = self.pf.encoder_response(
            Command.PAUSE_AUTONOMOUS, message['requestId'], ACK.SUCCESS)
        self.__client.write(response)
        # self._service.task_pause()

    def stop_autonomous(self, message):
        # rospy.loginfo("[TaskIO] stop autonomous %s " % message)
        print("[TaskIO] stop autonomous %s " % message)
        response = self.pf.encoder_response(Command.STOP_AUTONOMOUS, message['requestId'], ACK.SUCCESS)
        self.__client.write(response)
        # self._service.stop()

    def _remove_timer(self):
        if self._heartbeat_timeout_job:
            self.timer.remove_job(self._heartbeat_timeout_job)
        if self._report_car_job:
            self.timer.remove_job(self._report_car_job)
        if self._tele_report_job:
            self.timer.remove_job(self._tele_report_job)

    def on_disconnected(self, event):
        # rospy.loginfo("disconnected, {}".format(event))
        print("disconnected, {}".format(event))
        self._remove_timer()
        # self._service.stop()

    def on_connection_made(self, event):
        print("connected")
        self._send_login()

    def on_data_received(self, event):
        self.dispatch(event.data)

    # 向云端请求路口锁
    def on_request_crosslock(self, event):
        response = self.pf.encoder_request(Command.REQUEST_LOCK, event.data)
        print("[gateway:app.py] now request lock from cloud: {}".format(response))
        # rospy.loginfo("[gateway:app.py] now request lock from cloud: {}".format(response))
        self.__client.write(response)

    # 向云端释放路口锁
    def on_release_crosslock(self, event):
        response = self.pf.encoder_request(Command.RELEASE_LOCK, event.data)
        print("[gateway:app.py] now release lock to cloud: {}".format(response))
        # rospy.loginfo("[gateway:app.py] now release lock to cloud: {}".format(response))
        self.__client.write(response)

    def on_task_finished(self, event):
        print("task finished, {}".format(event))
        # rospy.loginfo("task finished, {}".format(event))
        self.timer.remove_job(self._report_task_job)
        self._report_task_job = None
        response = self.pf.encoder_request(Command.TASK_REAL_TIME_INFO, event.data)
        self.__client.write(response)

    def event_loop(self):
    # # 结束进程的时候删除生成的物体
        while True:
            try:
                event = self._event_bus.get(block=True)
                """
                queue().put()和queue().get()都有block参数
                对于queue().get(),当队列为空且block=True时,调用线程将阻塞,直到timeout时间后重试
                (此处,如果queue为空,那么程序将阻塞在当前这里,从而while不会无限制的占用资源)
                timeout为等待时间(s)
                """
                # self.client.update()
                func = self._event_handler_map.get(event.eventType)
                if func:
                    func(event)
                else:
                    print("event eventType: {}, data {} not processed".format(event.eventType, event.data))
            except Empty as e:
                pass
            except Exception as e:
                print("[app.py] got event unhandled error")
                # rospy.logerr(e)
        self.timer.shutdown()
        # self._web_server.shutdown()
        self.__client.shutdown()
        # self._service.stop()

    def run(self):
        self.client = Carla_Client()
        self.client.init()
        self.timer.start()
        self.__client.start()
        # self.monitor.start()
        # self._web_server.start()
        self.event_loop()

    def shutdown(self):
        self._event_bus.put(Event(EventType.InterruptedEvent, None))


if __name__ == '__main__':
    # rospy.loginfo("[app] send lane autonomous request")
    # outset = dict()
    # outset['name'] = "outset"
    # outset['x'] = 95.1965
    # outset['y'] = 0.914151
    # goal = dict()
    # goal['name'] = "goal"
    # goal['x'] = 0.657393
    # goal['y'] = 147.168
    # route = dict()
    # data = dict()
    # data['taskId'] = 1
    # data['outset'] = outset
    # data['goal'] = goal
    # data['type'] = ActionType.TRAJECTORY
    # data['route'] = route
    # print(json.dumps(data))
    app = App()
    try:
        app.run()
    except Exception:
        print("exception")

    # except rospy.ROSInterruptException, e:
    #     app.shutdown()
    #     rospy.loginfo(e)
