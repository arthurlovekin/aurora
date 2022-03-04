import math

def upperServoAngle(targetAngle):
    s = 3.2 # gimbal relative x
    t = -3.2 # gimbal relative y
    c = 1.5; # servo arm length
    b = s; # connector arm length
    a = -t + c; # TVC mount "arm" length
    # coordinates at the end of the TVC gimbal arm
    m = s - a * math.sin(targetAngle * math.pi / 180)
    n = t + a * math.cos(targetAngle * math.pi / 180)
    # quick commonly used vars
    m2 = m*m
    n2 = n*n
    b2 = b*b
    c2 = c*c
    m4 = m2*m2
    n4 = n2*n2
    b4 = b2*b2
    c4 = c2*c2
    p = (-b2*m-math.sqrt(-b4*n2 + 2*b2*c2*n2 + 2*b2*m2*n2 + 2*b2*n4 - c4*n2 + 2*c2*m2*n2 + 2*c2*n4 - m4*n2 - 2*m2*n4 - n2*n4)
    + c2*m + m*m2 + m*n2)/(2*(m2+n2))
    q = math.sqrt(c2-p*p)
    v = -math.atan2(p,q)*180/math.pi # angle that servo needs to rotate to
    return v + 90.0 # calibrated angle constant

def lowerServoAngle(targetAngle):
    s = 3.2 # gimbal relative x
    t = -2.9 # gimbal relative y
    c = 1.5; # servo arm length
    b = s; # connector arm length
    a = -t + c; # TVC mount "arm" length
    # coordinates at the end of the TVC gimbal arm
    m = s - a * math.sin(targetAngle * math.pi / 180)
    n = t + a * math.cos(targetAngle * math.pi / 180)
    # quick commonly used vars
    m2 = m*m
    n2 = n*n
    b2 = b*b
    c2 = c*c
    m4 = m2*m2
    n4 = n2*n2
    b4 = b2*b2
    c4 = c2*c2
    p = (-b2*m-math.sqrt(-b4*n2 + 2*b2*c2*n2 + 2*b2*m2*n2 + 2*b2*n4 - c4*n2 + 2*c2*m2*n2 + 2*c2*n4 - m4*n2 - 2*m2*n4 - n2*n4)
    + c2*m + m*m2 + m*n2)/(2*(m2+n2))
    q = math.sqrt(c2-p*p)
    v = -math.atan2(p,q)*180/math.pi # angle that servo needs to rotate to
    return v + 90.0 # calibrated angle constant