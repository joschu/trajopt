"""
Simple functions on numpy arrays
"""
from __future__ import division
import numpy as np

def interp2d(x,xp,yp):
    "Same as np.interp, but yp is 2d"
    yp = np.asarray(yp)
    assert yp.ndim == 2
    return np.array([np.interp(x,xp,col) for col in yp.T]).T
def normalize(x):
    return x / np.linalg.norm(x)
def normr(x):
    assert x.ndim == 2
    return x/norms(x,1)[:,None]
def normc(x):
    assert x.ndim == 2
    return x/norms(x,0)[None,:]
def norms(x,ax):
    return np.sqrt((x**2).sum(axis=ax))
def intround(x):
    return np.round(x).astype('int32')
def deriv(x):
    T = len(x)
    return interp2d(np.arange(T),np.arange(.5,T-.5),x[1:]-x[:-1])
def linspace2d(start,end,n):
    cols = [np.linspace(s,e,n) for (s,e) in zip(start,end)]
    return np.array(cols).T
def remove_duplicate_rows(mat):
    diffs = mat[1:] - mat[:-1]
    return mat[np.r_[True,(abs(diffs) >= 1e-5).any(axis=1)]]