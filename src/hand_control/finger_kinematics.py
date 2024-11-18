import numpy as np
import matplotlib.pyplot as plt
from .JointConfig import *
import os
from ament_index_python.packages import get_package_share_directory


# ------------------- Calculations of Tendon Lengths at single joint ------------------- #
# TODO: Add your own functions here to calculate the tendon lengths for each joint

def compute_r_T1T2(R, alpha1, alpha2, theta):
    # Convert angles to radians for trigonometric functions


    # Define the rotation matrices C10 and C12
    C10 = np.array([
        [np.cos(theta / 2), -np.sin(theta / 2)],
        [np.sin(theta / 2), np.cos(theta / 2)]
    ])
    
    C12 = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # Define vectors based on the given coordinates
    T1_vector = np.array([R * np.sin(alpha1), -R * np.cos(alpha1)])
    middle_vector = np.array([0, 2 * R])
    T2_vector = np.array([-R * np.sin(alpha2), R* np.cos(alpha2)])

    # Compute the transformation vector r_T1T2
    r_T1T2 = T1_vector + C10.dot(middle_vector) + C12.dot(T2_vector)
    
    return np.linalg.norm(r_T1T2)

def tendonlength_flexor_joint1_proximal_thumb(theta_joint1_proximal):
   '''Input: joint angle of joint1_proximal in rad
      Output: total normal lengths of flexor tendon through joint1_proximal
      sqrt((x2-x1)**2 + (y2-y1)**2) -2*x*y*np.cos(theta)'''
   #theta_joint1_proximal = 400 -> gets us 31 degrees = 0.541052rad
   #theta_joint1_proximal = 0 -> gets us 0 degrees = 0 rad
   #linear interpolation: return theta*(400/31)
   approximated_zero_distance = 0 #random
   return theta_joint1_proximal*(400/31)

def tendonlength_extensor_joint1_proximal_thumb(theta_joint1_proximal):
   '''Input: joint angle of joint1_proximal in rad
      Output: total normal lengths of flexor tendon through joint1_proximal
      sqrt((x2-x1)**2 + (y2-y1)**2) -2*x*y*np.cos(theta)'''
   approximated_zero_distance = 0 #random
   return approximated_zero_distance - theta_joint1_proximal

def tendonlength_flexor_joint2_distal_thumb(theta_joint2_distal):
   '''Input: joint angle of joint1_proximal in rad
      Output: total normal lengths of flexor tendon through joint1_proximal
      sqrt((x2-x1)**2 + (y2-y1)**2) -2*x*y*np.cos(theta)'''
   approximated_zero_distance = 0 #random
   return approximated_zero_distance + theta_joint2_distal

def tendonlength_extensor_joint2_distal_thumb(theta_joint2_distal):
   '''Input: joint angle of joint1_proximal in rad
      Output: total normal lengths of flexor tendon through joint1_proximal
      sqrt((x2-x1)**2 + (y2-y1)**2) -2*x*y*np.cos(theta)'''
   approximated_zero_distance = 0 #random
   return approximated_zero_distance + theta_joint2_distal

def pose2tendon_thumb(theta_joint1_proximal, theta_joint2_distal):
   return [tendonlength_flexor_joint1_proximal_thumb(theta_joint1_proximal), tendonlength_extensor_joint1_proximal_thumb(theta_joint1_proximal),
           tendonlength_flexor_joint2_distal_thumb(theta_joint2_distal), tendonlength_extensor_joint2_distal_thumb(theta_joint2_distal)]

#Flexion
def tendonlength_flexor_MCP(theta2, constants):
   '''Input: joint angle of joint1 in rad
      Output: total normal lengths of flexor tendon through joint1'''

   alpha1 = constants.alpha1
   alpha2 = constants.alpha2
   R = constants.R

   tendonlength = compute_r_T1T2(R, alpha1, alpha2, theta2)
   
   return tendonlength
#Extension
def tendonlength_extensor_MCP(theta2, constants):
   '''Input: joint angle of joint1 in rad
      Output: total normal lengths of extensor tendon through joint1'''

   alpha1 = constants.alpha1
   alpha2 = constants.alpha2
   R = constants.R

   tendonlength = compute_r_T1T2(R, alpha1, alpha2, theta2)
   
   return tendonlength
#right
def tendonlength_right_MCP(theta1, constants):
   '''Input: joint angle of joint2 in rad
      Output: total normal lengths of flexor tendon through joint2'''

   alpha1 = constants.alpha1
   alpha2 = constants.alpha2
   R = constants.R

   tendonlength = compute_r_T1T2(R, alpha1, alpha2, theta1)

   return tendonlength
#left
def tendonlength_left_MCP(theta1, constants):
   '''Input: joint angle of joint2 in rad
      Output: total normal lengths of extensor tendon through joint2'''
   alpha1 = constants.alpha1
   alpha2 = constants.alpha2
   R = constants.R

   tendonlength = compute_r_T1T2(R, alpha1, alpha2, theta1)
   
   return tendonlength

# ------------------- Calculations of Tendon Lengths for all joints ------------------- #
# TODO: Add your own functions here to calculate the tendon lengths for all joints and for each finger (if needed)

def DistalFlexor(theta_joint3):
   
      tendonlength = compute_r_T1T2(0.9, np.pi/4, 3 * np.pi/4, theta_joint3)
      return np.abs(tendonlength)

def DistalExtensor(theta_joint3):
   
      tendonlength = compute_r_T1T2(0.9, -np.pi/4, -3 * np.pi/4, theta_joint3)
      return np.abs(tendonlength)

def getDirection(Flexion_diff, Extension_diff, left_diff , right_diff):

   pullvector = [0,0]
   pushvector = [0,0]
   
   if Flexion_diff<Extension_diff:
      pullvector[1] = -1*Flexion_diff
      pushvector [1] = -1*Extension_diff
      direction1 = "Flexion"
      
   elif Flexion_diff>=Extension_diff:
      pullvector[1] = Extension_diff
      pushvector [1] = Flexion_diff
      direction1 = "Extension"
      

   if left_diff<right_diff:
      pullvector[0] = left_diff
      pushvector [0] = right_diff
      direction2 = "left"
      
   elif left_diff>=right_diff:
      pullvector[0] = -1*right_diff
      pushvector [0] = -1*left_diff
      direction2 = "right"

   return pullvector, pushvector, direction1, direction2

def getCoefficientsNew(Flexion, Extension, left, right):
   crf = 0 
   clf = 0
   cle = 0
   cre = 0
  

   rf = [1/np.sqrt(2), 1/np.sqrt(2)]
   re = [1/np.sqrt(2), -1/np.sqrt(2)]
   lf = [-1/np.sqrt(2), 1/np.sqrt(2)]
   le = [-1/np.sqrt(2), -1/np.sqrt(2)]

   FL = [left, Flexion]
   FR = [right, Flexion]
   EL = [-left, -Extension]
   ER = [right, -Extension]

   clf = np.dot(lf, FL)
   crf = np.dot(rf, FR)
   cre = np.dot(re, ER)
   cle = np.dot(le, EL)

   return np.array([crf, clf, cle, cre])

def getCoefficients(pullvector):
   crf = 0 
   clf = 0
   cle = 0
   cre = 0
   angle = np.arctan2(pullvector[1], pullvector[0])

   rf = [1/np.sqrt(2), 1/np.sqrt(2)]
   re = [1/np.sqrt(2), -1/np.sqrt(2)]
   lf = [-1/np.sqrt(2), 1/np.sqrt(2)]
   le = [-1/np.sqrt(2), -1/np.sqrt(2)]

   if (angle >= -np.pi/4 and angle <= np.pi/4):
      quadrant = 1
      crf = np.dot(pullvector, rf)
      cre = np.dot(pullvector, re)
   if (angle > np.pi/4 and angle <= 3* np.pi/4):
      quadrant = 2
      crf = np.dot(pullvector, rf)
      clf = np.dot(pullvector, lf)

   if (angle > 3*np.pi/4 or angle <= -3*np.pi/4):
      quadrant = 3

      clf = np.dot(pullvector, lf)
      cle = np.dot(pullvector, le)
   if (angle < -np.pi/4 and angle > -3*np.pi/4):
      quadrant = 4
      cle = np.dot(pullvector, le)
      cre = np.dot(pullvector, re)
   coeff = np.array([crf, clf, cle, cre])

   return quadrant, coeff


def getVects(coeff):

   rf = [1/np.sqrt(2), 1/np.sqrt(2)]
   re = [1/np.sqrt(2), -1/np.sqrt(2)]
   lf = [-1/np.sqrt(2), 1/np.sqrt(2)]
   le = [-1/np.sqrt(2), -1/np.sqrt(2)]

   basisVect = np.array([rf, lf, le, re])


   non_zero_indices = np.nonzero(coeff)[0]

   # Select the non-zero basis vectors and corresponding coefficients
   selected_basis_vectors = basisVect[non_zero_indices]
   selected_coefficients = coeff[non_zero_indices]

   result = None

   if len(selected_coefficients) is not 0:
      vec1 = (selected_coefficients[0]) * selected_basis_vectors[0]   
      vec2 = (selected_coefficients[1]) * selected_basis_vectors[1]
      result = [vec1, vec2]

   return result



def mapTendonlength(Flexion_diff, Extension_diff, left_diff , right_diff):

   pullvector, pushvector, direction1, direction2 = getDirection(Flexion_diff, Extension_diff, left_diff, right_diff)

   print("pullvec:", pullvector)
   

   quadrant, coeffpull = getCoefficients(pullvector)
   #quadrant, coeffpush = getCoefficients(pushvector)

   coeff = getCoefficientsNew(Flexion_diff, Extension_diff, left_diff, right_diff)

   print("coeffpull rf,lf,le,re:", coeffpull)
   #print("coeffpush rf,lf,le,re:", coeffpush)

   vects = getVects(coeffpull)

   plot(pullvector, pushvector, vects)

   return np.array(coeff) 

   
  
def plot(pullvector, pushvector, result):
   # Create regular plot with unit circle
   fig, ax = plt.subplots(figsize=(6,6))

   # Plot the unit circle
   unit_circle = plt.Circle((0, 0), 1, color='gray', fill=False, linestyle='--')
   ax.add_artist(unit_circle)

   # Plot the tendon vector
   if result is not None:
      for vect in result:
         print(vect)
         ax.quiver(0, 0, vect[0], vect[1], angles='xy', scale_units='xy', scale=1, color='r')
         

   ax.quiver(0, 0, pullvector[0], pullvector[1], angles='xy', scale_units='xy', scale=1, color='b')



   # Add diagonal lines for 45-degree positions
   diagonals = [[1/np.sqrt(2), 1/np.sqrt(2)], [-1/np.sqrt(2), 1/np.sqrt(2)], [-1/np.sqrt(2), -1/np.sqrt(2)], [1/np.sqrt(2), -1/np.sqrt(2)]]
   labels = ['rf', 'lf', 'le', 're']  # Annotations for each diagonal

   for i, (x, y) in enumerate(diagonals):
      ax.plot([0, x], [0, y], color='r', linestyle='--')  # Plot diagonal
      ax.text(1.2*x, 1.2*y, labels[i], horizontalalignment='center', fontsize=12, color='black')  # Annotate diagonal

   # Label axes for positive/negative x and y directions
   ax.text(0, 1.2, 'Flexion', horizontalalignment='center', fontsize=12, color='black')  # Positive Y
   ax.text(0, -1.2, 'Extension', horizontalalignment='center', fontsize=12, color='black')  # Negative Y
   ax.text(1.2, 0, 'Right', verticalalignment='center', fontsize=12, color='black')  # Positive X
   ax.text(-1.2, 0, 'Left', verticalalignment='center', fontsize=12, color='black')  # Negative X

   # Set equal aspect ratio and limits
   ax.set_aspect('equal')
   plt.xlim(-2.5, 2.5)
   plt.ylim(-2.5, 2.5)

   # Create second x-axis for theta2
   ax2 = ax.secondary_xaxis('top')
   ax2.set_xlabel('Theta2')
   ax2.set_ticks([-2, 0, 2])
   ax2.set_xticklabels(['0', '', r'$\pi$'])

   # Create second y-axis for theta1
   ax3 = ax.secondary_yaxis('right')
   ax3.set_ylabel('Theta1')
   ax3.set_ticks([2, 0, -2])
   ax3.set_yticklabels(['0', '', r'$\pi$'])

   # Show the plot
   plt.grid(True)
   plt.axhline(0, color='black',linewidth=0.5)
   plt.axvline(0, color='black',linewidth=0.5)
   plt.show()


def mapTendonlengthNew(Input):
   #Input: Flexion, left, right, extension
   #output: Flexion Left, Flexion right, left extension, right extension
   a1 = 1/np.sqrt(2)
   a2 = 0
   #a1=0.33
   #a2=0.87

   gamma1 = 0.56
   gamma2 = 0.83

   gamma = np.array([gamma1, gamma1, gamma2, gamma2])

  
   map =1/np.sqrt(2) * np.array([[1,1,0,0],
                  [1,0,1,0],
                  [0,1,0,1],
                  [0,0,1,1]])
   
   '''map = np.array([[a1,a1,-a2,-a2],
                  [a1,-a1,a2,-a2],
                  [-a1,a1,-a2,a2],
                  [-a1,-a1,a2,a2]])'''
   output = np.matmul(map, Input)

   return output

def pose2tendon_finger_indirect(theta1, theta2, theta3):
   '''Input: controllable joint angles
      Output: array of tendon lengths for given joint angles'''
   #Flexion, left, right, extension = getDifferences(current, target)


   roboticHand = create_robotic_hand_from_yaml(os.path.join(__file__, 'kinematics_constants.yaml'))
   #roboticHand = create_robotic_hand_from_yaml('real-world-robotics/controls/src/gripper_controller/config/kinematics_constants.yaml')

   kinematics_constants = roboticHand.finger
   
   MCPFlexion = tendonlength_flexor_MCP(theta2, kinematics_constants.MCPflexion)
   MCPExtension = tendonlength_extensor_MCP(theta2, kinematics_constants.MCPextension)
   MCPAdduction = tendonlength_left_MCP(theta1, kinematics_constants.MCPadduction)
   MCPAbduction = tendonlength_right_MCP(theta1, kinematics_constants. MCPabduction)

   print("MCPFlexion:", MCPFlexion, "MCPExtension:", MCPExtension, "MCPAdduction:", MCPAdduction, "MCPAbduction:", MCPAbduction)
   

   #tendolengthAbsolute = np.array(relativeTendonlength) 
   tendonLengthNew = mapTendonlengthNew(np.array([MCPFlexion, MCPAbduction, MCPAdduction, MCPExtension]))
   rightFlexor = tendonLengthNew[1]
   leftFlexor = tendonLengthNew[0]
   leftExtensor = tendonLengthNew[2]
   rightExtensor = tendonLengthNew[3]
   
   distalFlexor = DistalFlexor(theta3)
   distalExtensor =DistalExtensor(theta3)

   return [rightFlexor, leftExtensor, distalFlexor,distalExtensor ,leftFlexor, rightExtensor]
   

   
def main():  
   current1 = [0,0,0]
   target = [ -np.pi/2,0,0]
   print("homing")
   #homeTransformed = pose2tendon_finger_indirect(home, home)

   print("using differnce in tendons")
   currentTransformed = pose2tendon_finger_indirect(current1[0], current1[2], current1[2])
   new = pose2tendon_finger_indirect(target[0], target[1], target[2])
   DifferenceInTendon = np.array(new) - np.array(currentTransformed)
   #right flexor	left extensor	distal flexor	distal extensor	left flexor	 right extensor
   print("current", currentTransformed, "new", new)
   print("differnceintendons:", DifferenceInTendon)

if __name__ =="__main__":
   main()

