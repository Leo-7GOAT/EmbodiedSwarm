"""Minimal memory behavior tree; callbacks perform bounded work on each tick."""
from dataclasses import dataclass
from enum import Enum
import json
from math import isfinite
from time import monotonic


def _finite_number(number):
    try:
        return type(number) in (float,int) and isfinite(number)
    except OverflowError:
        return False


class Status(str,Enum):
    RUNNING="RUNNING"
    SUCCESS="SUCCESS"
    FAILURE="FAILURE"
    ABORTED="ABORTED"


@dataclass
class Context:
    blackboard: dict
    nodes: dict
    events: list
    clock: object
    tick_number: int
    check: object


class _TickInterrupted(Exception):
    pass


class Node:
    def __init__(self,name):
        if not isinstance(name,str) or not name:
            raise ValueError("nonempty node name required")
        self.name=name

    def descendants(self):
        return [self]

    def tick(self,context):
        context.check()
        memory=context.nodes.setdefault(self.name,{"status":"IDLE","data":{}})
        previous=memory["status"]
        if previous in (Status.SUCCESS.value,Status.FAILURE.value):
            return Status(previous)
        result=self.execute(context,memory["data"])
        context.check()
        if not isinstance(result,Status) or result==Status.ABORTED:
            raise ValueError("node must return RUNNING, SUCCESS or FAILURE")
        memory["status"]=result.value
        if previous!=result.value:
            context.events.append(dict(tick=context.tick_number,time=context.clock(),node=self.name,
                                       previous=previous,status=result.value))
        return result


class Action(Node):
    def __init__(self,name,on_tick):
        super().__init__(name);self.on_tick=on_tick

    def execute(self,context,memory):
        return self.on_tick(context,memory)


class Condition(Node):
    def __init__(self,name,predicate):
        super().__init__(name);self.predicate=predicate

    def execute(self,context,memory):
        value=self.predicate(context)
        if type(value) is not bool:
            raise ValueError("condition predicate must return bool")
        return Status.SUCCESS if value else Status.FAILURE


class Composite(Node):
    def __init__(self,name,children):
        super().__init__(name);self.children=list(children)
        if not self.children or any(not isinstance(child,Node) for child in self.children):
            raise ValueError("composite requires child nodes")

    def descendants(self):
        return [self]+[node for child in self.children for node in child.descendants()]


class Sequence(Composite):
    def execute(self,context,memory):
        index=memory.setdefault("cursor",0)
        while index<len(self.children):
            result=self.children[index].tick(context)
            if result!=Status.SUCCESS:
                return result
            index+=1;memory["cursor"]=index
        return Status.SUCCESS


class Fallback(Composite):
    def execute(self,context,memory):
        index=memory.setdefault("cursor",0)
        while index<len(self.children):
            result=self.children[index].tick(context)
            if result!=Status.FAILURE:
                return result
            index+=1;memory["cursor"]=index
        return Status.FAILURE


class BehaviorTree:
    def __init__(self,root,timeout_s=120.,clock=monotonic,abort=None,blackboard=None):
        if not isinstance(root,Node) or not _finite_number(timeout_s) or timeout_s<=0:
            raise ValueError("root node and finite positive timeout required")
        names=[node.name for node in root.descendants()]
        if len(names)!=len(set(names)):
            raise ValueError("unique node names required for persistent state")
        self.root,self.clock,self.abort=root,clock,abort or (lambda reason:None)
        self.names=names;self.nodes={};self.blackboard=dict(blackboard or {});self.events=[]
        now=clock()
        if not _finite_number(now) or not _finite_number(now+timeout_s):
            raise ValueError("finite clock required")
        self.deadline=now+timeout_s;self.last_clock=now;self.ticks=0
        self.status=Status.RUNNING;self.abort_reason=None;self.abort_requested=None

    def request_abort(self,reason="operator_abort"):
        if not isinstance(reason,str) or not reason:
            raise ValueError("nonempty abort reason required")
        self.abort_requested=reason

    def _stop(self,status,reason):
        self.status=status;self.abort_reason=reason
        self.abort(reason)
        now=self.clock()
        self.events.append(dict(tick=self.ticks,time=now if _finite_number(now) else self.last_clock,node="__tree__",status=status.value,reason=reason))
        return status

    def _check(self):
        now=self.clock()
        if not _finite_number(now) or now<self.last_clock:
            raise _TickInterrupted("invalid_clock")
        self.last_clock=now
        if self.abort_requested or now>=self.deadline:
            raise _TickInterrupted(self.abort_requested or "deadline")

    def tick(self):
        if self.status!=Status.RUNNING:
            return self.status
        now=self.clock()
        if not _finite_number(now) or now<self.last_clock:
            return self._stop(Status.ABORTED,"invalid_clock")
        self.last_clock=now
        if self.abort_requested:
            return self._stop(Status.ABORTED,self.abort_requested)
        if now>=self.deadline:
            return self._stop(Status.ABORTED,"deadline")
        self.ticks+=1
        context=Context(self.blackboard,self.nodes,self.events,self.clock,self.ticks,self._check)
        try:
            result=self.root.tick(context)
        except _TickInterrupted as interrupted:
            return self._stop(Status.ABORTED,str(interrupted))
        except Exception as exc:
            self.blackboard["exception"]=f"{type(exc).__name__}: {exc}"
            return self._stop(Status.ABORTED,"deadline" if self.clock()>=self.deadline else "action_exception")
        now=self.clock()
        if not _finite_number(now) or now<self.last_clock:
            return self._stop(Status.ABORTED,"invalid_clock")
        self.last_clock=now
        if self.abort_requested or now>=self.deadline:
            return self._stop(Status.ABORTED,self.abort_requested or "deadline")
        if result==Status.FAILURE:
            return self._stop(Status.FAILURE,"tree_failure")
        self.status=result
        return result

    def snapshot(self):
        value=dict(version=1,names=self.names,nodes=self.nodes,blackboard=self.blackboard,events=self.events,
                   deadline=self.deadline,last_clock=self.last_clock,ticks=self.ticks,status=self.status.value,
                   abort_reason=self.abort_reason,abort_requested=self.abort_requested)
        return json.loads(json.dumps(value,allow_nan=False))

    def restore(self,snapshot):
        value=json.loads(json.dumps(snapshot,allow_nan=False))
        if not isinstance(value,dict) or type(value.get("version")) is not int or value.get("version")!=1 or value.get("names")!=self.names:
            raise ValueError("checkpoint schema/tree mismatch")
        required={"nodes","blackboard","events","deadline","last_clock","ticks","status","abort_reason","abort_requested"}
        if not required.issubset(value):
            raise ValueError("incomplete checkpoint")
        now=self.clock()
        if not _finite_number(value["deadline"]) or not _finite_number(value["last_clock"]) or not _finite_number(now) or now<value["last_clock"]:
            raise ValueError("checkpoint requires the original monotonic clock domain")
        if type(value["ticks"]) is not int or value["ticks"]<0 or not isinstance(value["blackboard"],dict) or not isinstance(value["nodes"],dict) or not isinstance(value["events"],list):
            raise ValueError("invalid checkpoint counters or blackboard")
        if value["status"] not in [status.value for status in Status]:
            raise ValueError("invalid checkpoint tree status")
        for field in ("abort_reason","abort_requested"):
            if value[field] is not None and (not isinstance(value[field],str) or not value[field]):
                raise ValueError("invalid checkpoint abort reason")
        for event in value["events"]:
            if not isinstance(event,dict) or not {"tick","time","node","status"}.issubset(event):
                raise ValueError("invalid checkpoint event schema")
            if type(event["tick"]) is not int or not 0<=event["tick"]<=value["ticks"] or not _finite_number(event["time"]) or event["node"] not in self.names+["__tree__"] or event["status"] not in [status.value for status in Status]:
                raise ValueError("invalid checkpoint event values")
        for node in self.root.descendants():
            memory=value["nodes"].get(node.name)
            if node.name in value["nodes"]:
                if not isinstance(memory,dict) or memory.get("status") not in ("IDLE","RUNNING","SUCCESS","FAILURE") or not isinstance(memory.get("data"),dict):
                    raise ValueError("invalid checkpoint node state")
                if isinstance(node,Composite):
                    cursor=memory["data"].get("cursor",0)
                    if type(cursor) is not int or not 0<=cursor<=len(node.children):
                        raise ValueError("invalid composite cursor")
        if set(value["nodes"])-set(self.names):
            raise ValueError("checkpoint contains unknown nodes")
        status=Status(value["status"])
        self.nodes,self.blackboard,self.events=value["nodes"],value["blackboard"],value["events"]
        self.deadline,self.last_clock,self.ticks=value["deadline"],value["last_clock"],value["ticks"]
        self.status,self.abort_reason,self.abort_requested=status,value["abort_reason"],value["abort_requested"]
