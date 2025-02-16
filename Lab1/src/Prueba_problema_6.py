# Dependencies
import random
import numpy as np

## MATRIX

# Setting up the minimum size of the matrix
mSize = 0
while mSize < 5:

    mSize = int(input("Enter the size of the wishable size for the matrix: ")) # Asking user

    if mSize < 5:
        print("The matrix should be at least of 5 x 5") # Indicating user it should be from at least 5x5

# Setting up the matrixes

m = np.zeros((mSize, mSize), dtype=str) # main matrix
d = m # directions matrix (the one whc¡ich will contain the arrows)

# Start-up of the matrixes
for i in range(mSize):
    for j in range(mSize):
        m[i, j] = random.choice(['x', 'o', 'o'])

# Setting up the starting and finish points

m[0, 0] = 'R' # R of Robot
m[mSize - 1, mSize - 1] = 'D' # D of Destination

print("Initial Matrix:")
print(m)

## NAVIGATION

attempts = 0

if m[0, 1] == 'x' and m[1, 0] == 'x':
    print("No valid path exists from the beginning.") # Because it happened that it wanted to keep following

else:

    # Setting up special navigation parameters

    current_row, current_col = 0, 0
    last_origin_row = 0
    # If it hasn't got to the boundary then: 

    while (current_row, current_col) != (mSize - 1, mSize - 1):
        attempts += 1  # Increment attempts

        # RIGHT

        # It checks the following facts:
            # Is it inside a boundary in x? If yes, proceed.
            # Is there space left to the right or has it moved to that position before? If yes, proceed.
            # Is there an obstacle there?  If not, proceed.
        
        if current_col < mSize - 1 and (m[current_row][current_col + 1] == 'o' or m[current_row][current_col + 1] == 'M') and m[current_row][current_col + 1] != 'x':
            
            m[current_row][current_col] = 'M' # Writing in main matrix.
            d[current_row][current_col] = '→' # Writing in directions matrix.

            current_col += 1

            print(m)
            print("Moving right")
        
        # DOWN

        # It checks the following facts:
            # Is it inside a boundary in y? If yes, proceed.
            # Is there space left to the down side or has it moved to that position before? If yes, proceed.
            # Is there an obstacle there?  If not, proceed.

        elif current_row < mSize - 1 and (m[current_row + 1][current_col] == 'o' or m[current_row + 1][current_col] == 'M') and m[current_row + 1][current_col] != 'x':
            
            m[current_row][current_col] = 'M' # Writing in main matrix.
            d[current_row][current_col] = '↓' # Writing in directions matrix.

            current_row += 1

            print(m)
            print("Moving down")

        # If it is in the last row or last column and the destination is nearby, then proceed.

        elif (current_col == mSize - 1 or current_row == mSize - 1) and \
                ((current_row + 1 < mSize and m[current_row + 1][current_col] == 'D') or \
                (current_col + 1 < mSize and m[current_row][current_col + 1] == 'D')):
            
            # Move down or right if 'D' is at the destination and the robot is at the final column or row
            m[current_row][current_col] = 'M'

            # Reaching destination Down.
            if current_row + 1 < mSize and m[current_row + 1][current_col] == 'D':
                d[current_row][current_col] = '↓'  # Writing in directions matrix
                current_row += 1

                print(m)
                print("Moving to the destination down")

            # Reaching destination right.
            else:
                d[current_row][current_col] = '→'  # Writing in directions matrix
                current_col += 1

                print(m)
                print("Moving to the destination right")
        
        # None of the cases above (because the robot cannot continue in that path):

        else:

            last_origin_row += 1 #Updating this helper.

            if last_origin_row < mSize:  # Ensure the helper is within the matrix bounds (it gave problems when it wasn't).

                current_row, current_col = last_origin_row, 0 # Updating the parameters so the robot can begin again.

                print(m)
                print("Unable to move right or down. Stepping back to row after the origin and restarting.")

                print("Current position:", current_row, current_col) # Describing the new parameters.
            else:

                # In case there is no path to follow and we already reached the end of the matrix, there is nothing left to do so:

                print("No valid path exists.")
                break

    # But it the robot reached its destination (which is in the final boundaries), then print it, let's celebrate.

    if (current_row, current_col) == (mSize - 1, mSize - 1):
        print(m)
        print("Robot reached the destination!")

# Final printings

print("Final Matrix:")
print(d)
print("The robot did a total of: " + str(attempts) + " moves.")