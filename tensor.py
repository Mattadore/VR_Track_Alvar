#!/usr/bin/python
#just a helper script for getting urdf inertia values
import sys

def form(n):
    return '%5.5f' % n
    
if __name__ == "__main__":
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    density = float(sys.argv[4])
    volume = x*y*z
    mass = density*volume
    ixx = (mass/12.0)*(y*y+z*z)
    iyy = (mass/12.0)*(x*x+z*z)
    izz = (mass/12.0)*(y*y+x*x)
    print("Mass: " + form(mass))
    print("Inertia tensor:")
    print("|",form(ixx),form(0),form(0),"|",sep = " ")
    print("|",form(0),form(iyy),form(0),"|",sep = " ")
    print("|",form(0),form(0),form(izz),"|",sep = " ")

    print("In urdf:")
    print('<inertia ixx="{0}" ixy="0.0" ixz="0.0" iyy="{1}" iyz="0.0" izz="{2}"/>'.format(form(ixx),form(iyy),form(izz)))
    print('<mass value="{0}"/>'.format(form(mass)))
