import json
import numpy as np
import pytest
from embodied_swarm.task.bt import Action,BehaviorTree,Condition,Fallback,Sequence,Status


def test_sequence_memory_and_running_action_are_tickable():
    calls=[]
    def advance(context,memory):
        memory["progress"]=memory.get("progress",0)+1
        return Status.SUCCESS if memory["progress"]==3 else Status.RUNNING
    root=Sequence("root",[Condition("once",lambda context:calls.append("check") is None),Action("work",advance)])
    tree=BehaviorTree(root)
    assert tree.tick()==Status.RUNNING and tree.tick()==Status.RUNNING
    assert tree.tick()==Status.SUCCESS and tree.tick()==Status.SUCCESS
    assert calls==["check"] and tree.ticks==3
    assert tree.nodes["root"]["data"]["cursor"]==2


def test_fallback_failure_then_running_recovery():
    def recover(context,memory):
        context.blackboard["recovered"]=memory.get("started",False)
        memory["started"]=True
        return Status.SUCCESS if context.blackboard["recovered"] else Status.RUNNING
    root=Fallback("fallback",[Condition("fail",lambda context:False),Action("recover",recover)])
    tree=BehaviorTree(root)
    assert tree.tick()==Status.RUNNING
    assert tree.nodes["fallback"]["data"]["cursor"]==1
    assert tree.tick()==Status.SUCCESS and tree.blackboard["recovered"]


def test_abort_and_failure_stop_exactly_once():
    stops=[];calls=[]
    tree=BehaviorTree(Action("work",lambda context,memory:calls.append(1) or Status.RUNNING),abort=stops.append)
    tree.tick();tree.request_abort("test_stop")
    assert tree.tick()==Status.ABORTED and tree.tick()==Status.ABORTED
    assert stops==["test_stop"] and calls==[1]
    tree=BehaviorTree(Condition("fail",lambda context:False),abort=stops.append)
    assert tree.tick()==Status.FAILURE and stops[-1]=="tree_failure"


def test_deadline_checked_after_last_action_and_monotonic_clock():
    now=[0.];stops=[];later=[]
    def slow(context,memory):
        now[0]=2.
        return Status.SUCCESS
    tree=BehaviorTree(Sequence("sequence",[Action("slow",slow),Action("later",lambda context,memory:later.append(1) or Status.SUCCESS)]),timeout_s=1.,clock=lambda:now[0],abort=stops.append)
    assert tree.tick()==Status.ABORTED and stops==["deadline"]
    assert not later
    now[0]=0.;tree=BehaviorTree(Action("idle",lambda context,memory:Status.RUNNING),clock=lambda:now[0],abort=stops.append)
    now[0]=-1.
    assert tree.tick()==Status.ABORTED and stops[-1]=="invalid_clock"


def test_checkpoint_preserves_progress_blackboard_and_original_deadline():
    now=[0.]
    def action(context,memory):
        memory["count"]=memory.get("count",0)+1
        context.blackboard["count"]=memory["count"]
        now[0]+=.1
        return Status.SUCCESS if memory["count"]==3 else Status.RUNNING
    tree=BehaviorTree(Action("work",action),timeout_s=1.,clock=lambda:now[0])
    tree.tick();checkpoint=json.loads(json.dumps(tree.snapshot()))
    restored=BehaviorTree(Action("work",action),clock=lambda:now[0]);restored.restore(checkpoint)
    assert restored.deadline==1. and restored.tick()==Status.RUNNING
    assert restored.tick()==Status.SUCCESS and restored.blackboard["count"]==3
    assert restored.ticks==3
    wrong=BehaviorTree(Action("different",action),clock=lambda:now[0])
    with pytest.raises(ValueError):
        wrong.restore(checkpoint)
    checkpoint["blackboard"]["bad"]=np.nan
    with pytest.raises(ValueError):
        restored.restore(checkpoint)


def test_invalid_tree_callbacks_and_checkpoint_cursor_fail_safely():
    for timeout in (0.,np.nan,np.inf):
        with pytest.raises(ValueError):
            BehaviorTree(Action("work",lambda context,memory:Status.SUCCESS),timeout_s=timeout)
    with pytest.raises(ValueError):
        BehaviorTree(Sequence("root",[Condition("same",lambda context:True),Condition("same",lambda context:True)]))
    stops=[]
    tree=BehaviorTree(Action("bool_is_not_status",lambda context,memory:True),abort=stops.append)
    assert tree.tick()==Status.ABORTED and stops==["action_exception"]
    root=Sequence("seq",[Action("work",lambda context,memory:Status.RUNNING)])
    tree=BehaviorTree(root);tree.tick();saved=tree.snapshot();saved["nodes"]["seq"]["data"]["cursor"]=-1
    with pytest.raises(ValueError):
        tree.restore(saved)


@pytest.mark.parametrize("field,value",[
    ("deadline",True),("last_clock",False),("deadline",float("inf")),
    ("nodes",[]),("events",{}),("events",[None]),("events",[{"tick":0}]),
    ("nodes",{"work":None}),("nodes",{"work":[]}),
    ("blackboard",[]),("ticks",True),("status",[]),("abort_reason",5),
])
def test_invalid_checkpoint_schema_does_not_modify_running_tree(field,value):
    tree=BehaviorTree(Action("work",lambda context,memory:Status.RUNNING),clock=lambda:0.)
    tree.tick();before=tree.snapshot();bad=tree.snapshot();bad[field]=value
    with pytest.raises(ValueError):
        tree.restore(bad)
    assert tree.snapshot()==before


def test_checkpoint_restore_rejects_nonfinite_clock():
    now=[0.];tree=BehaviorTree(Action("work",lambda context,memory:Status.RUNNING),clock=lambda:now[0])
    tree.tick();saved=tree.snapshot();now[0]=np.nan
    with pytest.raises(ValueError):
        tree.restore(saved)
    assert tree.nodes==saved["nodes"] and tree.ticks==saved["ticks"]


@pytest.mark.parametrize("mode",["nominal","recover","exhaust","abort","deadline","resume"])
def test_real_bt_physics_recovery_and_safe_interruption(mode,tmp_path):
    from demos.bt_pick_place import run
    report=run(7,mode,tmp_path)
    assert report["status"]=="PASS_REAL_SIM"
    if mode in ("recover","resume"):
        assert report["recovery_success"] and report["grasp_checks"][0]["contacts"]==0
        assert report["grasp_checks"][1]["contacts"]>=2 and report["place_error_m"]<.04
        assert report["observations"][1]["time"]>report["observations"][0]["time"]
    if mode=="resume":
        assert report["checkpoint_restored"]
    if mode in ("abort","deadline","exhaust"):
        assert report["safety_latched_reason"] is not None and report["hold_drift_rad"]<.02
    if mode=="deadline":
        assert report["terminal_time_s"]<=3.004
