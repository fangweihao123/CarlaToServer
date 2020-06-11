#!/usr/bin/env python
# -*-coding:utf-8-*-
from twisted.internet.protocol import Protocol, ReconnectingClientFactory
from twisted.internet.endpoints import TCP4ClientEndpoint, connectProtocol
from twisted.internet import reactor
import time
import json
#from can_msgs.msg import ecu
from twisted.internet import reactor
from twisted.internet.protocol import ReconnectingClientFactory
from twisted.protocols.basic import LineReceiver
from enum import Enum, unique
import json


@unique
class ConnectionState(Enum):
    NEW = 0
    CONNECTING = 1
    CONNECTED = 2
    DISCONNECTED = 3


class lineReceiverProtocol(LineReceiver):
    def __init__(self):
        self.state = ConnectionState.DISCONNECTED

    def connectionMade(self):
        self.state = ConnectionState.CONNECTED

    def lineReceived(self, line):
        raise NotImplementedError

    def _write_thread_safe(self, msg):
        self.transport.write(msg)

    def write(self, data):
        reactor.callFromThread(self._write_thread_safe, data)
        # reactor.callFromThread(self.sendLine, data)
        return True

    # def sendControlMessage(self, msg):
    #     self.sendLine(json.dumps(msg))


class Connector(ReconnectingClientFactory, lineReceiverProtocol):
    def __init__(self, host, port):
        lineReceiverProtocol.__init__(self)
        self.__host = host
        self.__port = port

    def buildProtocol(self, addr):
        return self

    def lineReceived(self, line):
        print(line)

    def clientConnectionLost(self, connector, unused_reason):
        print("Connection lost, reason: ", unused_reason)
        ReconnectingClientFactory.clientConnectionLost(self, connector, unused_reason)

    def clientConnectionFailed(self, connector, reason):
        print("Connection establish failed, reson: ", reason)
        ReconnectingClientFactory.clientConnectionFailed(self, connector, reason)

    # def sendControlMessage(self, msg):
    #     self.sendLine()

    def start(self):
        reactor.connectTCP(self.__host, self.__port, self)
        import _thread
        self.__reactor_thread = _thread.start_new_thread(reactor.run, (False, ))

    def shutdown(self):
        reactor.stop()
        # TODO::结束处理,释放资源等
        return

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()


class App:
    def __init__(self):
        self._connector = Connector('127.0.0.1', 9005)

    def run(self):
        self._connector.start()  # 如果connector不使用多线程模式,将会阻塞在此
        counter = 0
        while counter < 100:
            time.sleep(0.1)
            counter += 1
            data = {}
            data['version'] = 0.3
            data['type'] = 0x23  # command type
            data['ack'] = 254
            data['requestId'] = counter
            data['vin'] = '112410001'
            message = {}
            message['speed'] = counter
            message['steer'] = 0.4
            message['brake'] = 0
            #message['shift'] = ecu.SHIFT_D
            message['shift'] = 1
            data['data'] = message
            msg = (json.dumps(data) + '\r\n').encode("utf-8")
            print("send data to server")
            self._connector.write(msg)
        print("send finished")


# 执行函数
if __name__ == '__main__':
    app = App()
    app.run()
