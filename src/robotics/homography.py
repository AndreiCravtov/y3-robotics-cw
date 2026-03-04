import numpy as np

# Functions to transform via the forward and inverse homography
def HtransformXYtoUV(H, xin, yin):
    xvec = np.array([xin, yin, 1])
    uvec = H.dot(xvec)
    uout = uvec[0]/uvec[2]
    vout = uvec[1]/uvec[2]
    return(uout, vout)

def HtransformUVtoXY(HInv, uin, vin):
    uvec = np.array([uin, vin, 1])
    xvec = HInv.dot(uvec)
    xout = xvec[0]/xvec[2]
    yout = xvec[1]/xvec[2]
    return(xout, yout)