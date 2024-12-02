import numpy as np
from typing import Dict

# the information of the tendons in the hand. Each tendon represents a grouped actuation.
GC_TENDONS = {
    # "right_thumb": {},
    # "right_thumb_joint_1": {},
    "thumb_pp2mp": {
        "thumb_pp2mp": 1,
        # "thumb_mp2dp_virt": 0.71,
    #     "thumb_mp2dp": 0.71,
    },
    "index_abd_virt": {"index_abd": 1},
    "root2index_pp_virt": {"root2index_pp": 1},
    "index_pp2mp_virt": {
        "index_pp2mp": 1,
        "index_mp2dp_virt": 0.71,
        "index_mp2dp": 0.71,
    },
    "middle_abd_virt": {"middle_abd": 1},
    "root2middle_pp_virt": {"root2middle_pp": 1},
    "middle_pp2mp_virt": {
        "middle_pp2mp": 1,
        "middle_mp2dp_virt": 0.71,
        "middle_mp2dp": 0.71,
    },
    "ring_abd_virt": {"ring_abd": 1},
    "root2ring_pp_virt": {"root2ring_pp": 1},
    "ring_pp2mp_virt": {"ring_pp2mp": 1, "ring_mp2dp_virt": 0.71, "ring_mp2dp": 0.71},
    "pinky_abd_virt": {"pinky_abd": 1},
    "root2pinky_pp_virt": {"root2pinky_pp": 1},
    "pinky_pp2mp_virt": {
        "pinky_pp2mp": 1,
        "pinky_mp2dp_virt": 0.71,
        "pinky_mp2dp": 0.71,
    },
}

# the mapping from fingername to the frame of the fingertip
# Use pytorch_kinematics.Chain.print_tree() to see the tip frame
FINGER_TO_TIP: Dict[str, str] = {
    "thumb": "thumb_mp",
    "index": "index_fingertip",
    "middle": "middle_fingertip",
    "ring": "ring_fingertip",
    "pinky": "pinky_fingertip",
}

# the mapping from fingername to the frame of the fingerbase (The base that fixed to the palm)
# Use pytorch_kinematics.Chain.print_tree() to see the base frame
FINGER_TO_BASE = {
    "thumb": "thumb_base",
    "index": "index_um_virt",
    "middle": "middle_um_virt",
    "ring": "ring_um_virt",
    "pinky": "pinky_um_virt",
}

GC_LIMITS_LOWER = np.array(
    [
        0.0,  # root2thumb_base
        # -95.0,  # thumb_base2pp
        # 0.0,  # thumb_pp2mp_virt
        -30.0,  # index_abd_virt
        0.0,  # root2index_pp_virt
        0.0,  # index_pp2mp_virt
        -30.0,  # middle_abd_virt
        0.0,  # root2middle_pp_virt
        0.0,  # middle_pp2mp_virt
        -30.0,  # ring_abd_virt
        0.0,  # root2ring_pp_virt
        0.0,  # ring_pp2mp_virt
        -30.0,  # pinky_abd_virt
        0.0,  # root2pinky_pp_virt
        0.0,  # pinky_pp2mp_virt
    ]
)
GC_LIMITS_UPPER = np.array(
    [
        130.0,  # root2thumb_base
        # 60.0,  # thumb_base2pp
        # 110.0,  # thumb_pp2mp_virt
        30.0,  # index_abd_virt
        95.0,  # root2index_pp_virt
        110.0,  # index_pp2mp_virt
        30.0,  # middle_abd_virt
        95.0,  # root2middle_pp_virt
        110.0,  # middle_pp2mp_virt
        30.0,  # ring_abd_virt
        95.0,  # root2ring_pp_virt
        110.0,  # ring_pp2mp_virt
        30.0,  # pinky_abd_virt
        95.0,  # root2pinky_pp_virt
        110.0,  # pinky_pp2mp_virt
    ]
)
