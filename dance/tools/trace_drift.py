"""Print base position and yaw over time from sim trace.npz files: trace_drift.py TRACE..."""
import sys, numpy as np
def yaw(q):
    w,x,y,z=q.T; return np.degrees(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
for path in sys.argv[1:]:
    t=np.load(path,allow_pickle=True); tm=t['time']; p=t['qpos']
    y=yaw(p[:,3:7]); print(path)
    for s in (0,3,5,7,10,13,16,20,25,30,35):
        i=np.searchsorted(tm,s)
        if i<len(tm): print(f"  t={tm[i]:5.1f} x={p[i,0]:6.2f} y={p[i,1]:6.2f} yaw={y[i]:7.2f}")
