from enum import IntEnum, Enum
import time

class TaskState(IntEnum):
    CREATED = 0x00
    RUNNING = 0x03
    FINISHED = 0x08
    INTERRUPTED = 0x10
    DEVICE_INTERRUPTED = 0x18
    KILLING = 0x30
    KILLED = 0x31

class Station:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.name = "default"

class WayPoint:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.v = 0
        self.t = 0

class TrjSection:
    def __init__(self):
        self.sectionId = 1


class SectionTask:
    """
    抽象SectionTask接口,具体的任务将继承SectionTask
    """

    @staticmethod
    def create_section_task(section_id, action):
        #将outset goal v 单独拿出来
        outset = Station()
        outset.name = str(action['outset']['name'])
        outset.x = float(action['outset']['x'])
        outset.y = float(action['outset']['y'])
        goal = Station()
        goal.name = str(action['goal']['name'])
        goal.x = float(action['goal']['x'])
        goal.y = float(action['goal']['y'])
        v = float(action['v'])
        # print action
        # print("action: type: %s action.trj : %s" % (action['type'], str(ActionType.TRAJECTORY)))
        return TrjSectionTask(section_id, v, outset, goal, action['points'])

    def __init__(self, section_id, speed, outset, goal):
        self.section_id = str(section_id)
        self.v = speed
        self.outset = outset
        self.goal = goal
        self.start_stamp = None
        self.end_stamp = None
        self.status = TaskState.CREATED
        self.percentage = 0

    def get_section_id(self):
        return self.section_id

    def get_speed(self):
        return self.v

    @property
    def start_stamp(self):
        return self.start_stamp

    @start_stamp.setter
    def start_stamp(self, stamp):
        self.start_stamp = stamp

    @property
    def end_stamp(self):
        return self.end_stamp

    @end_stamp.setter
    def end_stamp(self, stamp):
        self.end_stamp = stamp

    @property
    def state(self):
        return self.state

    @state.setter
    def state(self, state):
        self.state = state

    @property
    def percentage(self):
        return self.percentage

    @percentage.setter
    def percentage(self, percentage):
        self.percentage = percentage

class TrjSectionTask(SectionTask):
    """
    """

    def __init__(self, section_id, speed, outset, goal, points_json_o):
        # SectionTask.__init__(self, section_id, speed, outset, goal)
        self.waypoints = map(self.point_to_waypoint, points_json_o)

    @staticmethod
    def point_to_waypoint(point):
        """
        :type point dict
        :param point:
        :rtype WayPoint
        :return:
        """
        way_point = WayPoint()
        way_point.x = point['position']['x']
        way_point.y = point['position']['y']
        way_point.v = 0
        way_point.t = 0
        return way_point

    def to_trj_task(self):
        task = TrjSection()
        task.sectionId = self.section_id
        task.waypoints = self.waypoints
        task.outset = self.outset
        task.goal = self.goal
        return task


class DrivingTask:
    def __init__(self, task_id, speed, route_json_o):
        self.task_id = task_id
        self.speed = speed
        self.sections = self.to_sections(route_json_o)
        self.cur = -1
        self.state = TaskState.CREATED

    def get_task_id(self):
        return self.task_id

    def to_sections(self, route_json_o):
        """
        :param route_json_o:
        :type route_json_o json object
        :return:
        """
        sections = []
        for ind, action in enumerate(route_json_o):
            section = SectionTask.create_section_task(("%s_%d" % (self.task_id, ind)), action)
            if section:
                sections.append(section)
        return sections

    def get_state(self):
        """"""
        return self.state

    def get_task_state(self):
        """"""
        state = dict()
        state['taskId'] = self.task_id
        state['status'] = int(self.state)
        state['timestamp'] = int(time.time() * 1000)
        if self.cur == len(self.sections):
            self.cur = len(self.sections) - 1
        state['offset'] = self.cur
        state['percentage'] = self.sections[self.cur].percentage
        return state

    def move_next(self):
        self.cur += 1
        if self.cur == len(self.sections):
            return None
        return self.sections[self.cur]

    def get_cur_section_id(self):
        if 0 <= self.cur < len(self.sections):
            return self.sections[self.cur].section_id
        return None

    def set_state(self, state):
        self.state = state

    def set_section_state(self, section_id, section_state):
        """
        :param section_id:
        :type section_id str
        :param section_state:
        :type section_state TaskInfo
        :return:
        """
        for i, section in enumerate(self.sections):
            if section.section_id == section_id:
                section.state = section_state.state
                section.percentage = section_state.percentage
                if section.state == TaskState.FINISHED or section.state == TaskState.KILLED:
                    section.end_stamp = time.time()
                break

    def get_all_info(self):
        """
        collect all the section tasks.
        :return:
        """
        info = {
            "taskId": self.task_id,
            "v": self.speed,
            "state": self.state,
            "actions": []
        }
        for section in self.sections:
            action = {
                "actionId": section.section_id,
                "gmtCreate": section.start_stamp,
                "gmtEnd": section.end_stamp,
                "state": section.state,
                "v": section.get_speed(),
                "percentage": section.percentage
            }
            info["actions"].append(action)
        return info
