import math
import numpy as np

class Array3DAppender:
    def __init__(self):
        self.arrays = []

    def append_2b2(self, sample):
        """
        sample: 2x2 형태의 numpy 배열 또는 리스트
        """
        sample_arr = np.array(sample).reshape(1, 2, 2)
        self.arrays.append(sample_arr)

    def get_array(self):
        if len(self.arrays) == 0:
            return np.empty((0, 2, 2))
        return np.concatenate(self.arrays, axis=0)

# Vector operation functions
def v_add(x1, y1, x2, y2):
    return x1 + x2, y1 + y2

def v_sub(x1, y1, x2, y2):
    return x1 - x2, y1 - y2

def v_mul(x, y, scalar):
    return x * scalar, y * scalar

def v_div(x, y, scalar):
    return x / scalar, y / scalar

def v_length(x, y):
    return math.sqrt(x ** 2 + y ** 2)

def v_normalize(x, y):
    l = v_length(x, y)
    if l == 0:
        return 0, 0
    return x / l, y / l

# Coordinate transformation functions
def Cart2Polar(x, y):
    r = (x**2 + y**2)**0.5
    theta = math.atan2(y, x)
    return theta, r

def Polar2Cart(theta, r):
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

# Obstacle detection function
def detect_obstacles(th, d):
    points = Array3DAppender()
    dt_length = 1000

    f = False
    x1, x2, y1, y2 = None, None, None, None
    if d[0] > dt_length:
        for n in th:
            if not f:
                if d[n] < dt_length:
                    x1, y1 = th[n], d[n]
                    f = True
            else:
                if d[n] > dt_length:
                    x2, y2 = th[n], d[n]
                    f = False
                    points.append_2b2([[x1, y1], [x2, y2]])
    else:
        for n in th:
            if th[len(th) - n - 1] > dt_length:
                x1, y1 = th[len(th) - n - 1], d[len(th) - n - 1]
                f = True
                break
        for n in th:
            if th[n] > points[0][0][0]:
                break
            if not f:
                if d[n] < dt_length:
                    x1, y1 = th[n], d[n]
                    f = True
            else:
                if d[n] > dt_length:
                    x2, y2 = th[n], d[n]
                    f = False
                    points.append_2b2([[x1, y1], [x2, y2]])

    return points

# Main dodger function
def dodger_main(th, d):
    obstacles = detect_obstacles(th, d)
    # obstacles: (N, 2, 2) numpy array

    if obstacles.shape[0] == 0:
        # No obstacles
        return 0
    elif obstacles.shape[0] == 1 and obstacles[0][0][0] > obstacles[0][1][0]:
        # Obstacle on front
        return
    elif obstacles[len(obstacles) - 1][1][0] < 180:
        # Obstacles on Right
        return 
    elif obstacles[0][0][0] > 180 and obstacles[len(obstacles) - 1][1][0] < 360:
        # Obstacles on Left
        return 
    # elif obstacles.shape[0] == 1:
    #     mid_theta = (obstacles[0][0][0] + obstacles[0][1][0]) / 2
    #     mid_r = (obstacles[0][0][1] + obstacles[0][1][1]) / 2
    #     x, y = Polar2Cart(mid_theta, mid_r + 500)
    #     new_theta, _ = Cart2Polar(x, y)
    #     return new_theta
    # elif obstacles.shape[0] == 2:
    #     # Two obstacles
    #     mid1_theta = (obstacles[0][0][0] + obstacles[0][1][0]) / 2
    #     mid1_r = (obstacles[0][0][1] + obstacles[0][1][1]) / 2
    #     mid2_theta = (obstacles[1][0][0] + obstacles[1][1][0]) / 2
    #     mid2_r = (obstacles[1][0][1] + obstacles[1][1][1]) / 2

    #     x1, y1 = Polar2Cart(mid1_theta, mid1_r + 500)
    #     x2, y2 = Polar2Cart(mid2_theta, mid2_r + 500)

    #     mid_x, mid_y = v_div(v_add(x1, y1, x2, y2)[0], v_add(x1, y1, x2, y2)[1], 2)
    #     new_theta, _ = Cart2Polar(mid_x, mid_y)
    #     return new_theta