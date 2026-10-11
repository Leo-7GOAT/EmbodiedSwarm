"""Create a task scene while retaining the unmodified upstream Panda model."""
from pathlib import Path
import xml.etree.ElementTree as ET


def main():
    root = Path(__file__).resolve().parents[1]
    tree = ET.parse(root/"assets/franka_emika_panda/panda.xml")
    xml = tree.getroot()
    xml.find("compiler").set("meshdir", "../franka_emika_panda/assets")
    xml.find("option").set("timestep", "0.002")
    xml.remove(xml.find("keyframe"))
    hand = xml.find(".//body[@name='hand']")
    ET.SubElement(hand, "site", name="grasp_site", pos="0 0 0.1034", size="0.005", rgba="1 0 0 1")
    world = xml.find("worldbody")
    ET.SubElement(world, "light", pos="0 -1 3", dir="0 0 -1")
    ET.SubElement(world, "geom", name="floor", type="plane", size="2 2 .1", rgba=".75 .8 .85 1")
    cube = ET.SubElement(world, "body", name="object", pos="0.5 0 0.025")
    ET.SubElement(cube, "freejoint", name="object_free")
    ET.SubElement(cube, "geom", name="object_geom", type="box", size=".022 .022 .025", mass=".04",
                  friction="1.5 .01 .001", rgba=".85 .25 .1 1", solref=".005 1")
    ET.SubElement(world, "geom", name="target_marker", type="box", pos=".45 .22 .001",
                  size=".05 .05 .001", contype="0", conaffinity="0", rgba=".1 .8 .25 .7")
    path = root/"assets/mjcf/panda_pick_place.xml"
    tree.write(path, encoding="unicode")
    print(path)


if __name__ == "__main__":
    main()
