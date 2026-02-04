import re

# Test with various number formats that might appear
test_cases = [
    ("Standard", "-0.0003643878153525293"),
    ("Scientific", "1.0e-09"),
    ("Scientific uppercase", "1.0E-09"),
    ("Scientific positive", "1.0e+09"),
    ("Zero", "0.0"),
    ("Negative zero", "-0.0"),
]

pattern = r"([-\d.]+)"

for name, value in test_cases:
    test_str = f"x: {value}"
    match = re.search(f"x:\\s*{pattern}", test_str)
    if match:
        print(f"{name:20s}: MATCH - '{match.group(1)}'")
    else:
        print(f"{name:20s}: FAIL")

# Now test if scientific notation causes issues
print("\nTesting with scientific notation in message:")
output_with_sci = """position:
      x: 1.0e-09
      y: 2.0e-05
      z: -0.004393049981445074"""

pos_match = re.search(r"position:.*?x:\s*([-\d.]+).*?y:\s*([-\d.]+)", output_with_sci, re.DOTALL)
print("Match:", pos_match)
if pos_match:
    print("  x:", pos_match.group(1))
    print("  y:", pos_match.group(2))
