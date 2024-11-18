import yaml
import numpy as np
from sympy import symbols, pi, sympify

def convert_to_numerical(input_string):
    try:
        # Create a symbol for pi
        pi_symbol = symbols('pi_symbol')
        # Replace 'pi' with the custom symbol and then parse
        input_string = input_string.replace("pi", "pi_symbol")
        expression = sympify(input_string)
        # Substitute the custom pi_symbol with the numerical value of pi
        numerical_value = expression.subs(pi_symbol, pi).evalf()
        return float(numerical_value)
    except Exception as e:
        print(f"Error converting {input_string}: {e}")
        return None


# Define the FingerJoint class to hold joint data
class FingerJoint:
    def __init__(self, R=None, alpha1=None, alpha2=None):
        self.R = R
        self.alpha1 = convert_to_numerical(alpha1)
        self.alpha2 = convert_to_numerical(alpha2)

    def __repr__(self):
        return f"FingerJoint(R={self.R}, alpha1={self.alpha1}, alpha2={self.alpha2})"

# Define the Finger class to hold all joints for a finger
class Finger:
    def __init__(self, joints_data):
        self.MCPflexion = FingerJoint(**joints_data.get("MCPflexion", {}))
        self.MCPextension = FingerJoint(**joints_data.get("MCPextension", {}))
        self.MCPadduction = FingerJoint(**joints_data.get("MCPadduction", {}))
        self.MCPabduction = FingerJoint(**joints_data.get("MCPabduction", {}))
        self.PIPflexion = FingerJoint(**joints_data.get("PIPflexion", {}))
        self.PIPextension = FingerJoint(**joints_data.get("PIPextension", {}))

    def __repr__(self):
        return (f"Finger(MCPflexion={self.MCPflexion}, MCPextension={self.MCPextension}, "
                f"MCPadduction={self.MCPadduction}, MCPabduction={self.MCPabduction}, "
                f"PIPflexion={self.PIPflexion}, PIPextension={self.PIPextension})")

# Define the main class that holds all fingers
class RoboticHand:
    def __init__(self, yaml_data):
        self.finger = Finger(yaml_data.get("finger", {}))


    def __repr__(self):
        return (f"RoboticHand(finger1={self.finger})")

# Function to read YAML file and create RoboticHand instance
def create_robotic_hand_from_yaml(file_path):
    with open(file_path, 'r') as file:
        yaml_data = yaml.safe_load(file)
    return RoboticHand(yaml_data)

# Example usage
if __name__ == "__main__":
    hand = create_robotic_hand_from_yaml("real-world-robotics/controls/src/gripper_controller/config/kinematics_constants.yaml")
    print(hand.finger)
    