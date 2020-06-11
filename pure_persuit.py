import math

k = 0.1  # 前视距离系数
Lfc = 2.0  # 前视距离
Kp = 1.0  # 速度P控制器系数
dt = 0.1  # 时间间隔，单位：s
L = 2.9  # 车辆轴距，单位：m

def calc_target_index(state, cx, cy):
    # 搜索最临近的路点
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def pure_persuit_control(state,cx,cy,pind):
    ind = calc_target_index(state,cx,cy)
    if pind>= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
    # 后者为角度制 前者为弧度制
    alpha = math.atan2(ty - state.y, tx - state.x) - (state.yaw/180.0)*math.pi

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc
    #返回的是弧度  alpha应该为弧度制度
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta,ind
