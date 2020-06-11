#!/bin/python
# -*-encoding:utf-8-*-

from common import Timer

class Unit:
    def __init__(self, state, category):
        self._state = state
        self._category = category

    @property
    def state(self):
        return self._state

    @property
    def category(self):
        return self._category

    @state.setter
    def state(self, v):
        if isinstance(v, int):
            self._state = v
        else:
            raise ValueError()

    @category.setter
    def category(self, v):
        if isinstance(v, int):
            self._category = v
        else:
            raise ValueError()


class Monitor:
    def __init__(self, nodes, timer):
        """
        :type nodes: list of str
        :type timer: Timer
        """
        self._requestId = -1
        self._cur_nodes = dict()
        self._in_check = False
        self._timer = timer
        self._timer_handler = None
        self._car_values = dict()
        self._check_timer_id = None

    def start(self):
        self._check_timer_id = "Monitor_handler"
        self._timer.submit_periodical_job(self.check_health, 20, self._check_timer_id)

    def get_next_request_id(self):
        self._requestId += 1
        return self._requestId

    def _try_update(self):
        if len(self._nodes.keys()) != len(self._cur_nodes.keys()):
            return
        self._timer.remove_job(self._timer_handler)
        self._in_check = False
        self._cur_nodes = {}

    def _update_node(self, nodeName, status, category):
        self._nodes[nodeName] = Unit(status, category)

    def get_car_status(self):
        status = {}
        status["timestamp"] = int(rospy.Time.now().to_time() * 1000)
        status["pose"] = {
            "position": self._car_values.get("position"),
            "orientation": self._car_values.get("orientation")
        }
        status["v"] = self._car_values.get("v")
        status["wheelAngle"] = self._car_values.get("wheelAngle")
        status["gear"] = self._car_values.get("gear")
        status["nodes"] = []
        for node, unit in self._nodes.items():
            status["nodes"].append({
                "name": node,
                "status": unit.state,
                "type": unit.category
            })

        return status

    def get_modules(self):
        status = []
        for node, unit in self._nodes.items():
            status.append({
                "name": node,
                "status": unit.state,
                "type": unit.category
            })
        return status

    def get_module(self, module_name):
        for node, unit in self._nodes.items():
            if node == module_name:
                return {
                    "name": node,
                    "status": unit.state,
                    "type": unit.category
                }
        return {}
