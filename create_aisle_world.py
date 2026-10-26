from pathlib import Path
import xml.etree.ElementTree as ET

# =====================================================
# FILES
# =====================================================

worlds_dir = (
    Path.home()
    / "thesis_cart/src/intercart_comms/worlds"
)

source_file = worlds_dir / "intercart_world.world"
output_file = worlds_dir / "intercart_aisles.world"

# =====================================================
# DIMENSIONS (meters)
# =====================================================

WORLD_HALF_WIDTH = 6.0

AISLE_WIDTH = 1.8
SHELF_HEIGHT = 1.5

# The cross aisle is centered at (0, 0).
# Shelf blocks start at the outer edge of
# the 1.8-meter-wide open corridor.

half_aisle = AISLE_WIDTH / 2.0

block_size = WORLD_HALF_WIDTH - half_aisle

block_center = (
    WORLD_HALF_WIDTH + half_aisle
) / 2.0

# =====================================================
# LOAD YOUR EXISTING WORKING WORLD
# =====================================================

tree = ET.parse(source_file)
root = tree.getroot()

world = root.find("world")

if world is None:
    raise RuntimeError(
        "Could not find <world> in the original world."
    )

# Keep all existing plugins, lights, ground plane,
# and other world settings.

# =====================================================
# CREATE STATIC SHELF-BLOCK MODEL
# =====================================================

def add_shelf_block(name, x, y):
    model = ET.SubElement(
        world,
        "model",
        {"name": name}
    )

    ET.SubElement(model, "static").text = "true"

    ET.SubElement(model, "pose").text = (
        f"{x} {y} 0 0 0 0"
    )

    link = ET.SubElement(
        model,
        "link",
        {"name": "shelf_link"}
    )

    size = (
        f"{block_size} "
        f"{block_size} "
        f"{SHELF_HEIGHT}"
    )

    # Collision geometry:
    # prevents physical models from entering shelving
    collision = ET.SubElement(
        link,
        "collision",
        {"name": "shelf_collision"}
    )

    ET.SubElement(
        collision,
        "pose"
    ).text = f"0 0 {SHELF_HEIGHT / 2} 0 0 0"

    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size

    # Visual geometry:
    # shows the shelf-block in Gazebo
    visual = ET.SubElement(
        link,
        "visual",
        {"name": "shelf_visual"}
    )

    ET.SubElement(
        visual,
        "pose"
    ).text = f"0 0 {SHELF_HEIGHT / 2} 0 0 0"

    geometry_v = ET.SubElement(visual, "geometry")
    box_v = ET.SubElement(geometry_v, "box")
    ET.SubElement(box_v, "size").text = size

    material = ET.SubElement(visual, "material")

    ET.SubElement(
        material, "ambient"
    ).text = "0.45 0.53 0.60 1"

    ET.SubElement(
        material, "diffuse"
    ).text = "0.58 0.67 0.74 1"


# =====================================================
# FOUR CORNERS OF THE INTERSECTION
# =====================================================

for name, x, y in [
    ("shelf_NE",  block_center,  block_center),
    ("shelf_NW", -block_center,  block_center),
    ("shelf_SE",  block_center, -block_center),
    ("shelf_SW", -block_center, -block_center),
]:
    add_shelf_block(name, x, y)

# =====================================================
# SAVE
# =====================================================

ET.indent(tree, space="  ")

tree.write(
    output_file,
    encoding="utf-8",
    xml_declaration=True
)

print(f"World created: {output_file}")
print(f"Clear aisle width: {AISLE_WIDTH} m")
print(
    f"Central open intersection: "
    f"{AISLE_WIDTH} x {AISLE_WIDTH} m"
)
