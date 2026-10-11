import json, sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
base = Path('/mnt/c/Users/Leo/Desktop/project/EmbodiedSwarm/ros2/evidence/ci_38147716851/mujoco')
db = next(base.glob('bag_1791729366416875490/*.db3'))
connection = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
result = {}
for topic in ('/joint_states', '/mujoco/joint_states', '/mujoco/joint_commands'):
    rows = connection.execute('SELECT messages.timestamp,messages.data FROM messages JOIN topics ON messages.topic_id=topics.id WHERE topics.name=? ORDER BY messages.timestamp', (topic,)).fetchall()
    selected = []
    for stamp, payload in (rows[0], rows[-1]):
        message = deserialize_message(payload, JointState)
        selected.append(dict(record_stamp_ns=stamp, state=dict(zip(message.name,message.position))))
    result[topic] = dict(sample_count=len(rows),first_and_last=selected)
connection.close()
print(json.dumps(result, indent=2))
