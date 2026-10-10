"""Continuous CBS paths with progress dependencies at shared route sections.

A robot follows a whole permitted prefix, rather than stopping at each cell.
Only crossings depend on another robot's measured progress. New legs whose
routes overlap committed routes wait for admission; disjoint legs join without
canceling or replacing any existing action.
"""
from concurrent.futures import Future
from dataclasses import dataclass, field
import math
import threading
import time
from types import SimpleNamespace


def near(a, b, distance):
    return math.hypot(a[0]-b[0], a[1]-b[1]) < distance


def dependencies(routes, separation):
    """CBS time order becomes spatial-clearance prerequisites per waypoint."""
    result={name:[{} for _ in route.timed_waypoints] for name,route in routes.items()}
    names=list(routes)
    for ai,a in enumerate(names):
        for b in names[ai+1:]:
            gap=separation(a,b)
            for i,pa in enumerate(routes[a].timed_waypoints):
                for j,pb in enumerate(routes[b].timed_waypoints):
                    if not near(pa,pb,gap):continue
                    if i==j:
                        raise ValueError('CBS routes overlap at the same logical step')
                    later, index, earlier, required=(a,i,b,j+1) if i>j else (b,j,a,i+1)
                    result[later][index][earlier]=max(required,result[later][index].get(earlier,0))
    return result


def overlap(points, others, distance):
    return any(near(a,b,distance) for a in points for b in others)


class LegHandle:
    accepted=True
    def __init__(self,owner,name,target):
        self.owner,self.name,self.target=owner,name,target
        self.result=Future();self.waiting=True
    def get_result_async(self):return self.result
    def cancel_goal_async(self):
        future=Future()
        with self.owner.lock:
            self.owner.cancel_requests.append((self,future))
            self.owner.wake.set()
        return future


@dataclass
class Run:
    leg: LegHandle
    points: list
    requirements: list
    progress: int = -1
    end: int = -1
    action: object = None
    last_progress: float = field(default_factory=time.monotonic)
    last_pose: object = None
    done: bool = False


class CBSExecutor:
    def __init__(self,plan,send_step,pose,logger,step_timeout=30.0,separation=None):
        self.plan,self.send_step,self.pose,self.logger=plan,send_step,pose,logger
        self.step_timeout=float(step_timeout)
        self.separation=separation or (lambda a,b:.5)
        self.lock=threading.RLock();self.legs={};self.runs={}
        self.cancel_requests=[];self.wake=threading.Event();self.closed=False
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()

    def submit(self,name,target):
        with self.lock:
            if self.closed:return None,'cbs_executor_stopped'
            if name in self.legs:return None,'cbs_busy'
            leg=LegHandle(self,name,target);self.legs[name]=leg;self.wake.set()
            return leg,None

    def close(self):
        self.closed=True;self.wake.set();self.thread.join(timeout=6)

    def _resolve(self,legs,status):
        with self.lock:
            for name,leg in legs.items():
                if self.legs.get(name) is leg:self.legs.pop(name)
                if not leg.result.done():leg.result.set_result(SimpleNamespace(status=status))

    def _stop(self,actions):
        try:
            futures=[h.cancel_goal_async() for h in actions.values()]
            deadline=time.monotonic()+5
            while any(not f.done() for f in futures) and time.monotonic()<deadline:time.sleep(.02)
            if any(not f.done() for f in futures):return False
            for f in futures:f.result()
            terminal=[h.get_result_async() for h in actions.values()]
            while any(not f.done() for f in terminal) and time.monotonic()<deadline:time.sleep(.02)
            return all(f.done() for f in terminal)
        except Exception as exc:
            self.logger.error(f'CBS cancellation failed: {exc}');return False

    def _related(self,name):
        group={name}
        changed=True
        while changed:
            changed=False
            for n,run in self.runs.items():
                links=set().union(*(set(req) for req in run.requirements))
                if n in group or links & group:
                    expanded=group|{n}|links
                    if expanded!=group:group=expanded;changed=True
        return group

    def _fail_group(self,name,status=6):
        group=self._related(name)
        actions={n:r.action for n,r in self.runs.items() if n in group and r.action is not None}
        if not self._stop(actions):self.closed=True;return
        for n in group:
            run=self.runs.pop(n,None)
            if run is not None:self._resolve({n:run.leg},status)

    def _admit(self,pending):
        # Try a batch first (CBS can coordinate simultaneous conflicting legs).
        # If it intersects an existing committed route, try individual disjoint
        # legs instead. Queued robots remain physical parked obstacles in plan().
        batches=[pending]+([{n:leg} for n,leg in pending.items()] if len(pending)>1 else [])
        for batch in batches:
            batch={n:leg for n,leg in batch.items() if n not in self.runs and self.legs.get(n) is leg}
            if not batch:continue
            routes=self.plan({n:leg.target for n,leg in batch.items()})
            if not routes or any(n not in routes for n in batch):
                # A parked/moving teammate may temporarily block admission.
                # With no active actions, report a genuine planning failure.
                if not any(not r.done for r in self.runs.values()) and len(batch)==1:
                    self._resolve(batch,6)
                continue
            if any(overlap(routes[n].timed_waypoints,r.points[max(0,r.progress):],self.separation(n,other))
                   for n in batch for other,r in self.runs.items() if not r.done):continue
            # A rejected member must remain an obstacle: only admit full batches.
            requirements=dependencies({n:routes[n] for n in batch},self.separation)
            for n,leg in batch.items():
                self.runs[n]=Run(leg,list(routes[n].timed_waypoints),requirements[n])
            return

    def _allowed_end(self,run):
        end=run.progress
        for index in range(run.progress+1,len(run.points)):
            if any(other not in self.runs or self.runs[other].progress<required
                   for other,required in run.requirements[index].items()):break
            end=index
        return end

    def _update(self,name,run):
        xy=self.pose(name)
        if run.action is not None:
            now=time.monotonic()
            if xy is not None:
                old=run.last_pose
                if old is None or math.hypot(xy[0]-old[0],xy[1]-old[1])>.03 or (
                        len(xy)>2 and len(old)>2 and abs(math.atan2(math.sin(xy[2]-old[2]),math.cos(xy[2]-old[2])))>.1):
                    run.last_pose=xy;run.last_progress=now
                # Track actual progress through the already-authorized prefix.
                # A bounded forward window prevents jumping ahead at path loops.
                lo=max(0,run.progress);hi=min(run.end,lo+8)
                index=min(range(lo,hi+1),key=lambda i:math.hypot(xy[0]-run.points[i][0],xy[1]-run.points[i][1]))
                if near(xy,run.points[index],.10):run.progress=max(run.progress,index)
            future=run.action.get_result_async()
            if future.done():
                result=future.result()
                if result is None or result.status!=4:raise RuntimeError('FollowPath failed')
                if xy is None or not near(xy,run.points[run.end],.08):raise RuntimeError('success outside route endpoint tolerance')
                run.progress=run.end;run.action=None
                if run.end==len(run.points)-1:
                    run.done=True;run.progress=len(run.points)
                    self._resolve({name:run.leg},4)
                    return
            elif now-run.last_progress>self.step_timeout:raise RuntimeError('no motion during continuous FollowPath')
        if run.action is not None or run.done:return
        end=self._allowed_end(run)
        if end<=run.progress:
            run.leg.waiting=True;return
        # Duplicate CBS wait cells are governed by their prerequisites, not by
        # extra stop/start actions or a fleet-wide clock barrier.
        start=max(0,run.progress)
        points=run.points[start:end+1]
        if run.progress>=0 and all(p==points[0] for p in points) and end<len(run.points)-1:
            run.progress=end;return
        final=end==len(run.points)-1
        handle,error=self.send_step(name,points,run.leg.target,final)
        if error or handle is None:
            if error=='timeout':self.closed=True
            raise RuntimeError(f'continuous path rejected: {error}')
        run.action,run.end=handle,end
        run.last_progress=time.monotonic();run.last_pose=xy;run.leg.waiting=False

    def _run(self):
        next_admission=0
        try:
            while not self.closed:
                self.wake.wait(.05);self.wake.clear()
                with self.lock:
                    cancels,self.cancel_requests=self.cancel_requests,[]
                for leg,ack in cancels:
                    if self.legs.get(leg.name) is leg:
                        if leg.name in self.runs:self._fail_group(leg.name,5)
                        else:self._resolve({leg.name:leg},5)
                    if not ack.done():ack.set_result(SimpleNamespace())
                for name,run in list(self.runs.items()):
                    if run.done or self.runs.get(name) is not run:continue
                    try:self._update(name,run)
                    except Exception as exc:
                        self.logger.error(f'CBS {name} execution failed: {exc}')
                        self._fail_group(name)
                # Completed legs have cleared all their reservations. Remove
                # fulfilled prerequisites before admitting that robot's next leg.
                completed={n for n,r in self.runs.items() if r.done}
                for run in self.runs.values():
                    for requirement in run.requirements:
                        for n in completed:requirement.pop(n,None)
                for n in completed:self.runs.pop(n,None)
                with self.lock:pending={n:leg for n,leg in self.legs.items() if n not in self.runs}
                if pending and time.monotonic()>=next_admission:
                    try:self._admit(pending)
                    except Exception as exc:self.logger.error(f'CBS admission failed: {exc}')
                    next_admission=time.monotonic()+1.0
        finally:
            if not self._stop({n:r.action for n,r in self.runs.items() if r.action is not None}):
                self.logger.error('CBS shutdown: controller cancellation not confirmed')
            with self.lock:
                self._resolve(dict(self.legs),5)
                for _,ack in self.cancel_requests:
                    if not ack.done():ack.set_result(SimpleNamespace())
