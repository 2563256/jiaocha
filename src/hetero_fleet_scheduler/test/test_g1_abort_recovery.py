from types import SimpleNamespace
import pytest
from test_standoff import make_manager
from hetero_fleet_scheduler.task_model import Pose2D

@pytest.mark.parametrize('statuses,escape,kind,expected,calls',[
    ([6,4],.12,'g1_humanoid',True,1),
    ([6,6],.12,'g1_humanoid',False,1),
    ([6],0.,'g1_humanoid',False,1),
    ([6],None,'g1_humanoid',False,1),
    ([5],.12,'g1_humanoid',False,0),
    ([6],.12,'diff',False,0),
])
def test_aborted_g1_recovers_once_without_overlapping_navigation(statuses,escape,kind,expected,calls):
    m=make_manager();r=m.robots['r1'];r.type=kind
    terminal=iter(statuses);attempts=[];backups=[]
    m._send_leg_goal=lambda *args:(SimpleNamespace(get_result_async=lambda:None),None)
    def result(*args):
        status=next(terminal);attempts.append(status);return status,False,2.
    m._wait_for_result=result
    def backup(name):
        assert name not in m.active_goal
        backups.append(name);return escape
    m._short_backoff=backup
    ok,_,_=m._drive_leg(r,Pose2D(2.,0.,0.),'test')
    assert ok==expected and len(backups)==calls
