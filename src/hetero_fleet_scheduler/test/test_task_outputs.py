import csv,threading
from types import SimpleNamespace
from builtin_interfaces.msg import Time
from hetero_fleet_scheduler.metrics import MetricsRecorder
from test_standoff import make_manager
from hetero_fleet_scheduler.task_model import Task,TaskState,Pose2D

def test_completed_duplicate_route_has_separate_green_line_and_label():
 m=make_manager();m.robots={};m.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time()))
 m.get_parameter=lambda key:SimpleNamespace(value=[] if key=='status_panel_pos_overrides' else 1.)
 a=Task(id='a',pickup=Pose2D(10,-33.8,0),dropoff=Pose2D(10.22,-37.5,0));a.state=TaskState.COMPLETED
 b=Task(id='b',pickup=a.pickup,dropoff=a.dropoff);m.tasks={'a':a,'b':b};out=[];m.marker_pub=SimpleNamespace(publish=out.append)
 m._publish_markers();lines=[x for x in out[0].markers if x.ns=='tasks'];labels=[x for x in out[0].markers if x.ns=='task_labels']
 assert lines[0].points[0]!=lines[1].points[0]
 assert lines[0].color.g>lines[0].color.r and labels[0].color.g>labels[0].color.r
 assert labels[0].pose.position.z!=labels[1].pose.position.z

def test_report_writes_completion_without_timer_and_releases_fleet_lock(tmp_path):
 m=make_manager();m._metrics_io_lock=threading.Lock();m.metrics_dir=str(tmp_path);m.metrics=MetricsRecorder(list(m.robots))
 t=Task(id='done',pickup=Pose2D(0,0,0),dropoff=Pose2D(1,0,0));t.assigned_at_sec=1.;t.created_at_sec=1.
 m.metrics.note_submitted(1.);m.metrics.note_completed(t,m.robots['r1'],3.,1.,.5)
 m._report()
 with (tmp_path/'tasks.csv').open() as f:rows=list(csv.DictReader(f))
 assert rows[0]['task_id']=='done' and len(rows)==1
 assert m._lock.acquire(blocking=False);m._lock.release()

def test_output_error_does_not_prevent_other_update_or_fail_delivery():
 m=make_manager();calls=[]
 def broken():raise OSError('disk error')
 m._report=broken;m._publish_markers=lambda:calls.append('markers');m._sync_task_outputs()
 assert calls==['markers']
