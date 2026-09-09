class PIDController:
    def __init__(self, kP, kI, kD, min_, max_):
        self.Ie = 0
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.min_ = min_
        self.max_ = max_
    
    def step(e, de, h):
        self.Ie += e*h
        u = self.kP*e + self.kI*self.Ie + self.kD*de
        u = min(u, self.max_)
        if u<self.min_: u=0
        return u