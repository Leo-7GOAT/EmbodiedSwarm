"""Sequential task execution with retry, deadline, recovery and abort evidence."""
from dataclasses import dataclass, field
from time import monotonic
from math import isfinite


@dataclass
class TaskResult:
    success: bool
    completed: list = field(default_factory=list)
    failures: list = field(default_factory=list)
    recoveries: int = 0
    elapsed_s: float = 0.


class TaskExecutor:
    def __init__(self, stages, recovery, max_retries=1, timeout_s=120., clock=monotonic, abort=None):
        if not isinstance(max_retries,int) or max_retries < 0 or not isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("invalid retry/timeout policy")
        self.stages, self.recovery = stages, recovery
        self.max_retries, self.timeout_s, self.clock = max_retries, timeout_s, clock
        self.abort = abort or (lambda reason: None)

    def run(self):
        start = self.clock()
        result = TaskResult(False)
        def fail(reason):
            result.elapsed_s = self.clock()-start
            self.abort(reason)
            return result
        for name, action in self.stages:
            for attempt in range(self.max_retries+1):
                if self.clock()-start > self.timeout_s:
                    result.failures.append({"stage": name, "reason": "timeout", "attempt": attempt})
                    return fail("timeout")
                try:
                    if not action():
                        raise RuntimeError("success criterion not satisfied")
                    if self.clock()-start > self.timeout_s:
                        result.failures.append({"stage":name,"reason":"timeout","attempt":attempt})
                        return fail("timeout")
                    result.completed.append(name)
                    break
                except (RuntimeError, ValueError) as exc:
                    result.failures.append({"stage": name, "reason": str(exc), "attempt": attempt})
                    if attempt == self.max_retries:
                        return fail("retry_exhausted")
                    try:
                        self.recovery(name)
                    except (RuntimeError,ValueError) as recovery_error:
                        result.failures.append({"stage":name,"reason":"recovery_failed: "+str(recovery_error),"attempt":attempt})
                        return fail("recovery_failed")
                    result.recoveries += 1
                    if self.clock()-start > self.timeout_s:
                        result.failures.append({"stage":name,"reason":"timeout_during_recovery","attempt":attempt})
                        return fail("timeout")
        result.success = True
        result.elapsed_s = self.clock()-start
        return result
