Video Demo: https://youtu.be/7KSG-KsRqcc 
The project is incomplete, so the arm is missing the electrical magnet to pick up and set down metal checkers. The project is built using ESP-32 Thing, three SG90 servos, a 6V battery pack (four AA batteries), 3D-printed parts for the arm, and some wooden parts for the board and arm support.

Challenges and Design:
The biggest constraints were the 1.80 kg-cm torque and the 180 degrees of freedom of the provided SG90 servos. To accommodate that, I made the following design choices:
1. The base of the arm is located above the center of the board. This way the arm needs to be just long enough to reach the furthest squares at full extension at 45 degrees to all axes.
2. The "elbow" of the arm is operated by a servo that serves as a counter-balance to the rest of the arm, so the "shoulder" joint does not need as much torque.
3. The radial joint is geared 2:1, so 180 degrees of servo rotation is equivalent to 360 degrees of arm rotation. The "shoulder" and "elbow" joints of the arm needed maximum torque without sacrificing the required degree of freedom. The "shoulder" joint servo was geared 1:2 and "elbow" joint servo - 2:3.

Software:
The program takes a list of square positions as input, such as [A1, G5], and performs a motion of taking a piece from A1 to G5. The list of positions can be of arbitrary length, since checkers can jump multiple pieces at once.

Potential improvements:
The movement could be smoothed out using a smoother path of motion for the moved piece and constant acceleration. The code could be implemented more elegantly using ternary operations.
