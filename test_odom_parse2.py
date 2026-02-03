import re

# Test with full output including scientific notation
output = """header:
  stamp:
    sec: 1765362888
    nanosec: 836508080
  frame_id: odom_combined
child_frame_id: base_footprint
pose:
  pose:
    position:
      x: -0.0003643878153525293
      y: 0.5546483993530273
      z: -0.004393049981445074
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0021965232244525426
      w: 0.9999975876399525
  covariance:
  - 1.0e-09
  - 0.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"""

# Current regex patterns (might match wrong x,y if there are multiple)
pos_match = re.search(r"position:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+)", output, re.DOTALL)
quat_match = re.search(
    r"orientation:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+).*?z:\s*([-\d.]+).*?w:\s*([-\d.]+)",
    output, re.DOTALL
)

print("Position match:", pos_match)
if pos_match:
    print("  x:", pos_match.group(1))
    print("  y:", pos_match.group(2))
else:
    print("  FAILED!")

print("\nOrientation match:", quat_match)
if quat_match:
    print("  x:", quat_match.group(1))
    print("  y:", quat_match.group(2))
    print("  z:", quat_match.group(3))
    print("  w:", quat_match.group(4))
else:
    print("  FAILED!")

# Test if there could be confusion with twist linear
linear_match = re.search(r"linear:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+)", output, re.DOTALL)
print("\nLinear match:", linear_match)
if linear_match:
    print("  Groups:", linear_match.groups())
