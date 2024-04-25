# class robot:
#     a1 = 0.235 #m
#     a2 = 0.235
#     b1 = 0.33
#     b2 = 0.33
#     w = 0.25
#     working_mode = '++'
class robot:
    def __init__(self, a1, a2, b1, b2, w, working_mode='++'):
        self.a1 = a1
        self.a2 = a2
        self.b1 = b1
        self.b2 = b2
        self.w = w
        self.working_mode = working_mode
