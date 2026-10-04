class StateNode:
    def __init__(self, x, y, yaw, v, delta, g_cost=0.0, h_cost=0.0, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = delta

        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent = parent

    # def __eq__(self, other):
    #     print("StateNode __eq__")
    #     return (
    #         self.x == other.x
    #         and self.y == other.y
    #         and self.yaw == other.yaw
    #         and self.v == other.v
    #         and self.delta == other.delta
    #     )

    def __lt__(self, other):
        return (self.g_cost + self.h_cost) < (other.g_cost + other.h_cost)
