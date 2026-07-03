import numpy as np
import math

class Potential:
    def __init__(self, xc, yc):
        self.xc = xc
        self.yc = yc
        self.R = 64
        self.a = 4
        self.b = 6
        self.c = 4
        self.max_force = 5.0

    def alpha1(self, x, y, z=0):
        dx = x - self.xc
        dy = y - self.yc
        return self.a*dx**4 - self.b*dx**2*dy**2 + self.c*dy**4 - self.R

    def grad_alpha1(self, x, y, z=0):
        dx = x - self.xc
        dy = y - self.yc
        dadx = 4*self.a*dx**3 - 2*self.b*dx*dy**2
        dady = -2*self.b*dx**2*dy + 4*self.c*dy**3
        return [dadx, dady]

    def alpha2(self,z, x=0, y=0):
        return z
    def grad_alpha2(self,x, y, z=0):
        return [0.0, 0.0, 1.0]
    
    def V(self, x, y, z=0):
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(0)
        
        V = 1/2*a1*a1 + 1/2*a2*a2
        return V

    def grad_V (self, x, y, z=0.0):
        a1 = self.alpha1(x, y)
        a2 = self.alpha2(z)
        grad_a1 = self.grad_alpha1(x,y)
        grad_a2 = self.grad_alpha2(x,y, 0)

        dVdx= a1*grad_a1[0] + a2*grad_a2[0]
        dVdy = a1*grad_a1[1] + a2*grad_a2[1]

        return [dVdx, dVdy]
    
    def force(self, x, y, z=0):
        G  = 2.5

        dV = self.grad_V(x,y)  
        grad_a1 = self.grad_alpha1(x,y)

        H = 0.4
        fx_circ = H * (-grad_a1[1])
        fy_circ = H * ( grad_a1[0])

        fx = -G*dV[0]+ fx_circ
        fy = -G*dV[1] + fy_circ
        norm = math.hypot(fx, fy)

        # if norm > self.max_force:
        #     fx = fx / norm * self.max_force
        #     fy = fy / norm * self.max_force
        fx = fx
        fy = fy

        if fx > self.max_force:
            fx = self.max_force
        elif fx < -self.max_force:
            fx = -self.max_force

        if fy > self.max_force:
            fy = self.max_force
        elif fy < -self.max_force:
            fy = -self.max_force
        
        return [fx, fy]