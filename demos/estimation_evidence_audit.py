"""Verify strict JSON, finite metrics and reversible malformed-sensor evidence."""
import hashlib
import json
import math
from pathlib import Path


def audit(root=Path("artifacts/estimation")):
    def reject_constant(value):
        raise ValueError(f"nonstandard numeric JSON token: {value}")
    def finite_numbers(value):
        if isinstance(value,dict):
            for item in value.values():
                finite_numbers(item)
        elif isinstance(value,list):
            for item in value:
                finite_numbers(item)
        elif isinstance(value,float) and not math.isfinite(value):
            raise ValueError("nonfinite decoded numeric value")
    files=[];marked=0
    for path in sorted(root.iterdir()):
        if path.suffix not in (".json",".jsonl") or path.name=="evidence_audit.json":
            continue
        content=path.read_bytes()
        lines=content.decode().splitlines() if path.suffix==".jsonl" else [content.decode()]
        for line in lines:
            value=json.loads(line,parse_constant=reject_constant)
            finite_numbers(value)
            if isinstance(value,dict) and "nonfinite_fields" in value:
                if value["nonfinite_fields"]!={"packet.gyro_z":"NaN"} or value["packet"]["gyro_z"] is not None:
                    raise ValueError("malformed sensor provenance cannot be restored")
                marked+=1
        files.append(dict(path=path.name,bytes=len(content),records=len(lines),sha256=hashlib.sha256(content).hexdigest()))
    benchmark=json.loads((root/"benchmark.json").read_text(),parse_constant=reject_constant)
    runs=benchmark["results"]
    if len(runs)!=25 or any(r["status"]!="PASS_REAL_SIM" for r in runs):
        raise ValueError("benchmark acceptance failed")
    navigation=sum(r["navigation_status"]=="PASS_REAL_SIM" for r in runs)
    aborts=sum(r["navigation_status"]=="ABORTED" for r in runs)
    if navigation!=20 or aborts!=5 or marked!=5:
        raise ValueError("navigation/abort/provenance evidence count mismatch")
    result=dict(status="PASS_STRICT_EVIDENCE",json_files=len(files),records=sum(f["records"] for f in files),
                navigation_successes=navigation,deliberate_safe_aborts=aborts,restorable_nan_packets=marked,files=files)
    (root/"evidence_audit.json").write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in result.items() if k!="files"},indent=2))
    return result


if __name__=="__main__":
    audit()
