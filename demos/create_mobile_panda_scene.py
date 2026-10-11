"""Mount the upstream Panda on a force-actuated planar constrained base."""
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    tree = ET.parse(root/"assets/mjcf/panda_pick_place.xml")
    xml = tree.getroot()
    world = xml.find("worldbody")
    panda = world.find("body[@name='link0']")
    world.remove(panda)
    base = ET.SubElement(world,"body",name="mobile_base",pos="0 0 .13")
    for name, kind, axis in (("base_x","slide","1 0 0"),("base_y","slide","0 1 0"),("base_yaw","hinge","0 0 1")):
        ET.SubElement(base,"joint",name=name,type=kind,axis=axis,damping="1")
    ET.SubElement(base,"geom",name="base_geom",type="box",size=".18 .14 .1",mass="12",rgba=".1 .3 .6 1")
    panda.set("pos","0 0 .1")
    base.append(panda)
    world.find("body[@name='object']").set("pos","1 0 .025")
    world.find("geom[@name='target_marker']").set("pos",".95 .22 .001")
    act = xml.find("actuator")
    for name, gain, force in (("base_x",350,200),("base_y",350,200),("base_yaw",75,50)):
        ET.SubElement(act,"velocity",name=name+"_velocity",joint=name,kv=str(gain),forcerange=f"-{force} {force}")
    path = root/"assets/mjcf/mobile_panda.xml"
    tree.write(path,encoding="unicode")
    print(path)


if __name__ == "__main__":
    main()
