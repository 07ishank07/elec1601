from pathlib import Path
import re, math
from collections import deque
S=(Path(__file__).resolve().parents[1]/'advanced/last.cpp').read_text()
def num(n): return float(re.search(r'const (?:float|int) '+n+r' = ([0-9.]+)',S)[1])
def arr(n): return [int(v.strip(),0) for v in re.search(n+r'\[MAP_N \+ 1\] = \{([^}]+)',S)[1].split(',')]
H,V=arr('MAP_H'),arr('MAP_V')
# Independent transcription of orange walls in image; bit 0 is left/top.
assert H==[63,4,29,13,42,4,63]
assert V==[63,42,18,0,54,21,63]
DX,DY=[0,1,0,-1],[-1,0,1,0]
def wall(m,c,r,d):
    assert 0<=c<6 and 0<=r<6,(m,c,r,d)
    if m:
        c=5-c
        if d in (1,3):d=4-d
    return bool((H[r]>>c if d==0 else H[r+1]>>c if d==2 else V[c]>>r if d==3 else V[c+1]>>r)&1)
def goal(m,c,r):
    opens=[d for d in range(4) if not wall(m,c,r,d)]
    if len(opens)!=1:return False
    d=opens[0];b=(c+DX[d],r+DY[d]);s1=(d+1)%4;s2=(d+3)%4
    if not wall(m,*b,s1) or not wall(m,*b,s2) or wall(m,*b,d):return False
    z=(b[0]+DX[d],b[1]+DY[d])
    return not(wall(m,*z,s1) and wall(m,*z,s2))
def distances(m):
    dist={};q=deque()
    for c in range(6):
        for r in range(6):
            if goal(m,c,r):dist[c,r]=0;q.append((c,r))
    while q:
        c,r=q.popleft()
        for d in range(4):
            if wall(m,c,r,d):continue
            n=c+DX[d],r+DY[d]
            if n not in dist:dist[n]=dist[c,r]+1;q.append(n)
    return dist
DIST=[distances(0),distances(1)]
class Run:
    def __init__(self,c,r,h):
        self.c,self.r,self.h=c,r,h
        self.x=self.y=8;self.hr=0;self.info={};self.moved=False
        self.cands=[(m,c,r,h) for m in range(2) for c in range(6) for r in range(6) for h in range(4)]
        self.time=5.;self.moves=0;self.turns=0;self.trace=[]
    def choose(self):
        if len(self.cands)==1:
            m,c,r,h=self.cands[0];here=DIST[m].get((c,r),255)
            if here not in (0,255):
                for rel in (0,1,3,2):
                    d=(h+rel)%4
                    if not wall(m,c,r,d) and DIST[m].get((c+DX[d],r+DY[d]))==here-1:return rel
        info=self.info.get((self.x,self.y),0)
        for rel in (0,1,3):
            d=(self.hr+rel)%4;nx,ny=self.x+DX[d],self.y+DY[d]
            if not(info>>d&1) and 0<=nx<16 and 0<=ny<16 and not(self.info.get((nx,ny),0)&16):return rel
        if info&128:return ((info>>5&3)-self.hr)&3
        if not(info>>((self.hr+2)&3)&1):return 2
        self.info={k:v&15 for k,v in self.info.items()};self.info[self.x,self.y]|=16
        for rel in (0,1,3):
            if not(info>>((self.hr+rel)%4)&1):return rel
        return 2
    def step(self):
        obs=[wall(0,self.c,self.r,(self.h+k)%4) for k in (3,0,1)]
        self.time+=.03+(.06 if not self.moved else 0)
        info=self.info.get((self.x,self.y),0)
        for rel,w in zip((3,0,1),obs):
            d=(self.hr+rel)%4
            if w:info|=1<<d
            else:info&=~(1<<d)
        if self.moved:
            back=(self.hr+2)%4;info&=~(1<<back)
            if not(info&16):info|=128|(back<<5)
        self.info[self.x,self.y]=info|16
        self.cands=[(m,c,r,h) for m,c,r,h in self.cands if [wall(m,c,r,(h+k)%4) for k in (3,0,1)]==obs]
        assert self.cands
        self.trace.append((self.c,self.r,self.h,len(self.cands),round(self.time,2)))
        if all(obs) and all(goal(m,c,r) for m,c,r,h in self.cands):return True
        for attempt in range(4):
            rel=self.choose()
            if rel:
                self.h=(self.h+rel)%4;self.hr=(self.hr+rel)%4
                self.cands=[(m,c,r,(h+rel)%4) for m,c,r,h in self.cands]
                speed=num('ANTICLOCKWISE_SPEED_CM_S') if rel==3 else num('CLOCKWISE_SPEED_CM_S')
                self.time+=int(math.pi*num('WHEEL_TRACK_CM')*(180 if rel==2 else 90)/(360*speed)*950)/1000+.1
                self.turns+=1
            self.time+=.03
            if not wall(0,self.c,self.r,self.h):break
            self.info[self.x,self.y]|=1<<self.hr
            self.cands=[(m,c,r,h) for m,c,r,h in self.cands if wall(m,c,r,h)]
            assert self.cands
        if wall(0,self.c,self.r,self.h):raise RuntimeError('Driving into wall')
        self.c+=DX[self.h];self.r+=DY[self.h]
        self.x+=DX[self.hr];self.y+=DY[self.hr];self.moved=True;self.moves+=1
        self.cands=[(m,c+DX[h],r+DY[h],h) for m,c,r,h in self.cands if not wall(m,c,r,h)]
        self.time+=int(num('CELL_CM')/num('FORWARD_SPEED_CM_S')*1000)/1000+.11
        return False

def main():
    rows=[];traces=[]
    for name,c,r in [('NW',0,0),('NE',5,0),('SW',0,5),('SE',5,5)]:
        for h in range(4):
            run=Run(c,r,h)
            for _ in range(500):
                if run.step():break
            else:raise RuntimeError('No finish in 500 steps')
            rows.append(f'| {name} | {"NESW"[h]} | {run.moves} | {run.turns} | {run.time:.2f} | {"within budget" if run.time<=150 else "OVER 150 s"} |')
            traces.append(f'{name}/{"NESW"[h]}: {run.trace}')
    report='# Advanced maze ideal cell-level simulation\n\n'
    report+='Python port of navigation/localisation, NOT execution of the complete C++ sketch.\n'
    report+='Map arrays match photo. Assumes exact cell-centre starts, perfect walls, perfect turns and cell travel.\n'
    report+='Optimistic times include startup 5 s, nominal drives, turns, pauses and minimum sensor reads.\n'
    report+='Exclude serial stalls, loop overshoot, extra alignment and all sensor/physical errors.\n'
    report+='Goal cells (zero-based column,row): '+str([k for k,v in DIST[0].items() if v==0])+'\n\n'
    report+='| Corner | Heading | Moves | Turns | Seconds | Result |\n|---|---|---:|---:|---:|---|\n'+'\n'.join(rows)
    report+='\n\nTrace tuples: column,row,heading,candidate count,seconds\n\n'+'\n\n'.join(traces)+'\n'
    (Path(__file__).parent/'advanced_maze_results.md').write_text(report)
    print(report.split('Trace tuples')[0])
if __name__=='__main__':main()
