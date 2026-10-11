import json, sqlite3, sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import String
base=Path(sys.argv[1]) / 'mujoco'
db=next(base.glob('bag_1791730847297859438/*.db3'))
c=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
r={}
for topic in ('/joint_states','/mujoco/joint_states','/hardware/safety_status'):
 rows=c.execute('SELECT messages.timestamp,messages.data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name=? ORDER BY messages.timestamp',(topic,)).fetchall()
 items=[]
 for stamp,payload in rows[:2]+rows[-2:]:
  m=deserialize_message(payload,String if topic.endswith('status') else JointState)
  items.append(dict(record_stamp_ns=stamp,data=m.data) if topic.endswith('status') else dict(record_stamp_ns=stamp,header_stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,state=dict(zip(m.name,m.position))))
 r[topic]=dict(count=len(rows),samples=items)
print(json.dumps(r,indent=2))
