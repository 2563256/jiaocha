"""Execution checks: preserve waits/barriers and stop before replanning."""
from concurrent.futures import Future
from types import SimpleNamespace
import time

from hetero_fleet_scheduler.cbs_execution import CBSExecutor


class Logger:
    def __init__(self): self.errors = []
    def error(self, text): self.errors.append(text)


def ready(value):
    f = Future(); f.set_result(value); return f


class Action:
    def __init__(self, pose, name, point, completed=False):
        self.pose, self.name, self.point = pose, name, point
        self.future = Future(); self.canceled = False
        if completed: self.finish()
    def finish(self):
        self.pose[self.name] = self.point
        self.future.set_result(SimpleNamespace(status=4))
    def get_result_async(self): return self.future
    def cancel_goal_async(self):
        self.canceled = True
        if not self.future.done(): self.future.set_result(SimpleNamespace(status=5))
        return ready(SimpleNamespace())


def until(predicate, timeout=2):
    deadline=time.monotonic()+timeout
    while not predicate() and time.monotonic()<deadline: time.sleep(.01)
    assert predicate()


def test_continuous_path_finishes_without_waiting_for_unrelated_robot():
    poses={'a':(0,0),'b':(2,0)};sent=[]
    paths={'a':[(0,i*.2) for i in range(101)],'b':[(2,0),(2,.2)]}
    def send(n,points,t,final):
        action=Action(poses,n,points[-1],completed=n=='a')
        sent.append((n,points,action));return action,None
    worker=CBSExecutor(lambda ts:{n:SimpleNamespace(timed_waypoints=paths[n]) for n in ts},send,poses.get,Logger())
    try:
        a,_=worker.submit('a',None);b,_=worker.submit('b',None)
        until(a.result.done)
        assert a.result.result().status==4 and not b.result.done()
        assert len([x for x in sent if x[0]=='a'])==1
        assert len(sent[0][1])==101
    finally:worker.close()


def test_crossing_waits_until_other_robot_clears_reserved_section():
    from hetero_fleet_scheduler.cbs_execution import dependencies
    routes={'a':SimpleNamespace(timed_waypoints=[(-1,0),(0,0),(1,0),(2,0)]),
            'b':SimpleNamespace(timed_waypoints=[(0,-2),(0,-1),(0,-1),(0,0),(0,1)])}
    req=dependencies(routes,lambda a,b:.5)
    assert req['b'][3]=={'a':2}
    poses={'a':(-1,0),'b':(0,-2)};sent=[]
    def send(n,points,t,final):
        action=Action(poses,n,points[-1]);sent.append((n,points,action));return action,None
    worker=CBSExecutor(lambda ts:{n:routes[n] for n in ts},send,poses.get,Logger())
    try:
        a,_=worker.submit('a',None);b,_=worker.submit('b',None)
        until(lambda:len(sent)==2)
        b_action=next(x for x in sent if x[0]=='b');assert b_action[1][-1]==(0,-1)
        b_action[2].finish();time.sleep(.15)
        assert len(sent)==2
        next(x[2] for x in sent if x[0]=='a').finish()
        until(lambda:len(sent)==3)
        sent[-1][2].finish();until(b.result.done)
        assert a.result.result().status==b.result.result().status==4
    finally:worker.close()


def test_new_disjoint_leg_does_not_cancel_running_action():
    poses={'a':(0,0),'b':(2,0)};sent=[];plans=[]
    def plan(ts):
        plans.append(set(ts));return {n:SimpleNamespace(timed_waypoints=[poses[n],(poses[n][0],.2)]) for n in ts}
    def send(n,points,t,f):
        action=Action(poses,n,points[-1]);sent.append(action);return action,None
    worker=CBSExecutor(plan,send,poses.get,Logger())
    try:
        worker.submit('a',None);until(lambda:len(sent)==1)
        worker.submit('b',None);until(lambda:len(sent)==2)
        assert plans[-1]=={'b'} and not sent[0].canceled
    finally:worker.close()


def test_no_plan_fails_instead_of_dispatching_independent_navigation():
    sent=[];worker=CBSExecutor(lambda _: {},lambda *args:sent.append(args),lambda _: (0,0),Logger())
    try:
        leg,_=worker.submit('a',None);until(leg.result.done)
        assert leg.result.result().status==6 and sent==[]
    finally:worker.close()


def test_step_timeout_cancels_all_running_actions():
    poses={'a':(0,0)};sent=[]
    def send(n,p,t,f):
        a=Action(poses,n,p[-1]);sent.append(a);return a,None
    worker=CBSExecutor(lambda _: {'a':SimpleNamespace(timed_waypoints=[(0,0),(.2,0)])},send,poses.get,Logger(),step_timeout=.05)
    try:
        leg,_=worker.submit('a',None);until(leg.result.done)
        assert sent[0].canceled and leg.result.result().status==6
    finally:worker.close()


def test_conflicting_new_leg_is_queued_without_canceling_existing_leg():
    poses={'a':(0,0),'b':(1,-1)};sent=[]
    paths={'a':[(0,0),(1,0),(2,0)],'b':[(1,-1),(1,0),(1,1)]}
    def send(n,points,t,f):
        action=Action(poses,n,points[-1]);sent.append(action);return action,None
    worker=CBSExecutor(lambda ts:{n:SimpleNamespace(timed_waypoints=paths[n]) for n in ts},send,poses.get,Logger())
    try:
        a,_=worker.submit('a',None);until(lambda:len(sent)==1)
        b,_=worker.submit('b',None);time.sleep(1.2)
        assert len(sent)==1 and not sent[0].canceled and b.waiting
        sent[0].finish();until(lambda:len(sent)==2)
        sent[1].finish();until(b.result.done)
        assert a.result.result().status==b.result.result().status==4
    finally:worker.close()


def test_completed_cancel_reply_does_not_fail_while_another_robot_is_canceling():
    """Nav2 returns no goals_canceling when a waypoint completed just before cancel."""
    import threading
    logger=Logger()
    worker=CBSExecutor(lambda _: {},lambda *a:None,lambda _: (0,0),logger)
    poses={'a':(0,0),'b':(2,0)}
    done=Action(poses,'a',(0,0),completed=True)
    moving=Action(poses,'b',(2,.2))
    done.cancel_goal_async=lambda:ready(SimpleNamespace(goals_canceling=[]))
    def cancel_moving():
        moving.canceled=True
        def terminal():moving.future.set_result(SimpleNamespace(status=5))
        threading.Timer(.05,terminal).start()
        return ready(SimpleNamespace(goals_canceling=[object()]))
    moving.cancel_goal_async=cancel_moving
    try:
        assert worker._stop({'a':done,'b':moving}) is True
        assert moving.future.done()
        assert not logger.errors
    finally:worker.close()


def test_planned_head_on_and_doorway_dependencies_can_finish():
    import numpy as np
    from hetero_fleet_scheduler.cbs import MapfGrid,cbs
    from hetero_fleet_scheduler.cbs_execution import dependencies
    for rows,starts,goals in [(['.....','.....'],[(0,0),(4,0)],[(4,0),(0,0)]),
                             (['...#...','...#...','.......'],[(1,0),(5,1)],[(5,1),(1,0)])]:
        grid=MapfGrid(occupied=np.array([[c=='#' for c in row] for row in rows]),res=1.,origin=(0.,0.),cell=1.)
        agents=[dict(name=n,grid=grid,mask=grid.mask(0.),start=s,goal=g,moving=True)
                for n,s,g in zip(('a','b'),starts,goals)]
        paths,_=cbs(agents,time_limit=5.)
        assert paths is not None
        routes={n:SimpleNamespace(timed_waypoints=p) for n,p in zip(('a','b'),paths)}
        requirements=dependencies(routes,lambda a,b:.5)
        progress={n:-1 for n in routes}
        for _ in range(sum(map(len,paths))+1):
            changed=False
            for n,r in routes.items():
                i=progress[n]+1
                if i<len(r.timed_waypoints) and all(progress[o]>=needed for o,needed in requirements[n][i].items()):
                    progress[n]=i;changed=True
                    if i==len(r.timed_waypoints)-1:progress[n]+=1
            if not changed:break
        assert all(progress[n]==len(r.timed_waypoints) for n,r in routes.items())
