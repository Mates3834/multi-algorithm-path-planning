import math
import numpy as np


def _wrap(a):
    return (a + math.pi)%(2*math.pi)-math.pi


def track_path(path, dt=0.1, speed=2.0, lookahead=4.0,
               max_yaw_rate=0.5, max_steps=3000):
    path=np.asarray(path,dtype=float)
    x,y=path[0]
    psi=math.atan2(path[1,1]-path[0,1],path[1,0]-path[0,0])
    traj=[]

    for _ in range(max_steps):
        traj.append([x,y,psi])
        d=np.linalg.norm(path[:,:2]-np.array([x,y]),axis=1)
        nearest=int(np.argmin(d))

        target_idx=nearest
        acc=0.0
        for i in range(nearest,len(path)-1):
            acc += np.linalg.norm(path[i+1]-path[i])
            target_idx=i+1
            if acc>=lookahead: break

        tx,ty=path[target_idx]
        desired=math.atan2(ty-y,tx-x)
        err=_wrap(desired-psi)
        yaw_rate=np.clip(1.2*err,-max_yaw_rate,max_yaw_rate)

        psi += yaw_rate*dt
        x += speed*math.cos(psi)*dt
        y += speed*math.sin(psi)*dt

        if np.linalg.norm(np.array([x,y])-path[-1])<1.5:
            break

    return np.asarray(traj)


def tracking_rmse(path,traj):
    P=np.asarray(path); T=np.asarray(traj)[:,:2]
    errs=[]
    for q in T:
        errs.append(np.min(np.linalg.norm(P-q,axis=1)))
    return float(np.sqrt(np.mean(np.square(errs))))
