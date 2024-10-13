import cv2
import mediapipe as mp
import time
import math

# Initialize Mediapipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

# Video capture using OpenCV
cap = cv2.VideoCapture(0)  # 0 for default camera

# Set up the timer
start_time = time.time()
interval = 0.2
recognition_threshold = 15
m = 0.143053
c = -0.013


def inversely_map_val(value, from_low, from_high, to_low, to_high):
    # Ensure the value is within the original range
    value = max(from_low, min(value, from_high))
    # Calculate the mapped value inversely
    mapped_value = to_low + (to_high - to_low) - (value - from_low) * (to_high - to_low) / (from_high - from_low)
    return mapped_value



def map_val(value, from_low, from_high, to_low, to_high):

    # Ensure the value is within the original range
    value = max(from_low, min(value, from_high))

    # Calculate the mapped value
    mapped_value = to_low + (value - from_low) * (to_high - to_low) / (from_high - from_low)

    return mapped_value
def is_palm(hand_coordinates, recognition_threshold, hand_side):

    percentage_error = 0
    if hand_side == "right":
        target_angles = [-2.52545687852896, -1.9483040078665688, -1.6103067854398168, -1.7002641084906414, 0.4339406264284531, -1.6126579050081975, -1.5747849194347137, -1.5298671963002544, 1.3646553259549035, -1.6218483263299037, -1.5968216501477754, -1.597339725176179, 1.3905249169753788, -1.5950140621519084, -1.5936207638922135, -1.6183316125880256, 1.3856806181487065, -1.560501843787499, -1.6247245652958373, -1.6636383731265538]

    elif hand_side == "left":
        target_angles = [-0.6471584741095117, -1.2338149080123801, -1.5680533711070337, -1.3390927308736622, 2.5762135399244217, -1.5333480331588123, -1.6020179945648463, -1.6217731619466849, 1.7657050993126022, -1.5262688124548656, -1.552629543476536, -1.53839238787029, 1.7539772707880155, -1.5342519286040701, -1.4900642516486404, -1.4658452828809432, 1.7811441058091946, -1.5726404662857638, -1.465885433114545, -1.4255914565456953]

    angles = [] 
    for i in range(21):
        if i != 20:
            x = hand_coordinates[i][0]
            y = hand_coordinates[i][1]
            x_next = hand_coordinates[i+1][0]
            y_next = hand_coordinates[i+1][1]

            angle = math.atan2(y_next-y, x_next-x)
            angles.append(angle)
    #print(angles)

    for j in range(21):
        if j != 20:
            percentage_error += abs((target_angles[j] - angles[j])/target_angles[j]) * 100

    if percentage_error/20 <= recognition_threshold:
        return True
    else:
        return False


while True:
    # Read frames from the camera
    ret, frame = cap.read()

    # Flip the frame horizontally
    frame = cv2.flip(frame, 1)

    # Convert the BGR image to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process the image and get hand landmarks
    results = hands.process(rgb_frame)

    # Check if hand landmarks are detected
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Extract and print the coordinates of all hand joints
            hand_coordinates = [(landmark.x, landmark.y, landmark.z) for landmark in hand_landmarks.landmark]
            #print(hand_coordinates[8])
            #print(hand_coordinates)
            if is_palm(hand_coordinates, recognition_threshold, hand_side="right") == True:
                relative_distance = 0
                # Draw the coordinates on the frame
                for i in range(4, 20, 4):
                    x, y, _ = hand_coordinates[i]
                    x_next, y_next, _ = hand_coordinates[i + 4]

                    relative_distance += math.sqrt((x_next - x) ** 2 + (y_next - y) ** 2)
                
                actual_distance = (1/relative_distance)*m+c
                x_distance  = map_val(hand_coordinates[9][0], 0.5, 0.85, -0.4, 0.4)
                #z_distance = 0.3
                z_distance  = inversely_map_val(hand_coordinates[9][1], 0.3, 0.7, 0.05, 0.4)
                #print(x_distance)
                # Map the actual distance value
                y_distance = map_val(actual_distance, 0.3, 0.6, 1, -1)

                print(x_distance, y_distance, z_distance)
                text = f"Actual Distance: {actual_distance:.4f}"
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                #print(map_val(actual_distance, 0.3, 0.6, 1, -1))
                
    # Check the timer and print every 0.2 seconds
    if time.time() - start_time >= interval:
        start_time = time.time()

    # Display the frame
    cv2.imshow("Hand Tracking", frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
