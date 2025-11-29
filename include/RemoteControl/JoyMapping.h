"""
Nintendo Pro Controller - ROS2 Joy Mapping
Final mapping based on actual physical button testing

axes (6 total):
  0: Left Stick X    (Left is +1, Right is -1)
  1: Left Stick Y    (Up is +1, Down is -1)
  2: Right Stick X   (Left is +1, Right is -1)
  3: Right Stick Y   (Up is +1, Down is -1)
  4: D-Pad X         (Left is +1, Right is -1) - digital but in axes
  5: D-Pad Y         (Up is +1, Down is -1) - digital but in axes

buttons (16 total):
  0:  B
  1:  A
  2:  Y
  3:  X
  4:  L (Left Bumper)
  5:  R (Right Bumper)
  6:  L2 (Left Trigger - digital press)
  7:  R2 (Right Trigger - digital press)
  8:  Minus (-)
  9:  Plus (+)
  10: L Stick Button (Press Left Stick)
  11: R Stick Button (Press Right Stick)
  12: Home
  13: Star (Screenshot)
  14: (unused)
  15: (unused)
"""

# Python dictionary for easy reference
JOY_AXES = {
    'LEFT_STICK_X': 0,      # Left is +1, Right is -1
    'LEFT_STICK_Y': 1,      # Up is +1, Down is -1
    'RIGHT_STICK_X': 2,     # Left is +1, Right is -1
    'RIGHT_STICK_Y': 3,     # Up is +1, Down is -1
    'DPAD_X': 4,            # Left is +1, Right is -1 (digital)
    'DPAD_Y': 5,            # Up is +1, Down is -1 (digital)
}

JOY_BUTTONS = {
    'B': 0,
    'A': 1,
    'Y': 2,
    'X': 3,
    'L': 4,               # Left Bumper
    'R': 5,               # Right Bumper
    'L2': 6,              # Left Trigger
    'R2': 7,              # Right Trigger
    'MINUS': 8,           # Minus button (-)
    'PLUS': 9,            # Plus button (+)
    'L_STICK_BUTTON': 10, # Press left stick
    'R_STICK_BUTTON': 11, # Press right stick
    'HOME': 12,
    'STAR': 13,           # Screenshot
    # 14-15 unused
}

# Reverse mappings for easy lookup
AXES_NAMES = {v: k for k, v in JOY_AXES.items()}
BUTTON_NAMES = {v: k for k, v in JOY_BUTTONS.items()}
