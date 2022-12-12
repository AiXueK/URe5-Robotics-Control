15%; due in Week4 Lab
---
- [Spec](#spec)
  - [PartA: Trace Digit (4%)](#parta-trace-digit-4)
    - [Requirment](#requirment)
    - [Steps](#steps)
  - [PartA: Advanced Task: Trace digits in random order (6%)](#parta-advanced-task-trace-digits-in-random-order-6)
  - [PartB: Translation and Rotation (2%)](#partb-translation-and-rotation-2)
  - [PartC: Mimicking a human (2%)](#partc-mimicking-a-human-2)
- [Problem](#problem)
  - [Debug the translation and rotation point](#debug-the-translation-and-rotation-point)
  - [Base Frame too large](#base-frame-too-large)
- [Question](#question)

---
# Spec
![](top%20down%20view%20of%20the%20table.png)

![](example%20TCP%20path.png)
## PartA: Trace Digit (4%)
### Requirment
1. Write on a horizontal table **60mm** above the talble
2. Final joint facing directly down
3. Write out 0~9 in order
4. Start and end in home position
5. The bottom left hand corner of the A4 sheet of paper will be placed at the following x,y,z position [-588.53, -133.30, 0] (vertically below the home position)

### Steps
1. In home position
2. Print out the digits to write in console
3. Write the digits in order
4. Print out "Program Complete"

## PartA: Advanced Task: Trace digits in random order (6%)
1. The others are the same
2. Traces out 10 random digits

## PartB: Translation and Rotation (2%)
1. A plane is parallel to the table with a rotation about z-axis (e.g.X = -300mm, Y = -300mm RZ = - 30 deg). In base frame cooridinate, this coordinate will indicate the bottom left corner of the A4 page.
2. Perform translation and rotation to that plane

## PartC: Mimicking a human (2%)
1. Stroke: move like a person (including transition to next digit)
2. Smooth: constant velocity and smooth motions without jerking motions.

# Problem
## Debug the translation and rotation point
1. Before the debug
  ![](problem%20value%20image.png)

## Base Frame too large
1. Need to find a way to change the cm to mm unit

# Question
1. Lucida Console of size 100 will be used. This is a monospace font. If you don’t have this font, don’t worry, try to find any monospaced font and use that at size 100. The font must not be a 7-segment style font, it must have curves in the relevant numbers?
    
    is it a requirement for the size of the digit?

2. The random digit in parta advanced task:
   1. Are the numbers within 0~9
   2. Can we preprogram the 0~9 and reorder them at the spot?