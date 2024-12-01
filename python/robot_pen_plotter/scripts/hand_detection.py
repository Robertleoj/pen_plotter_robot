import cv2
import numpy as np
import math
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils


INDEX_FINGER_TIP = 8
THUMB_TIP = 4

DISTANCE_THRESHOLD = 0.05

drawn_lines: list[list[np.ndarray]] = []
curr_drawn_line = []


def main():
    global drawn_lines
    global curr_drawn_line

    cap = cv2.VideoCapture(0)
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )

    while True:
        ret, frame = cap.read()
        # ummirror
        frame = cv2.flip(frame, 1)

        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for id, lm in enumerate(hand_landmarks.landmark):
                    h, w, c = frame.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)

                    print(lm.x, lm.y)

                    color = (255, 0, 255)
                    radius = 3

                    if id == INDEX_FINGER_TIP:
                        color = (0, 255, 0)
                        radius = 5

                    elif id == THUMB_TIP:
                        color = (0, 0, 255)
                        radius = 5

                    cv2.circle(frame, (cx, cy), radius, color, cv2.FILLED)

                    # draw the id
                    cv2.putText(
                        frame,
                        str(id),
                        (cx, cy),
                        cv2.FONT_HERSHEY_PLAIN,
                        2,
                        (0, 0, 0),
                        2,
                    )

                index_finger_tip = hand_landmarks.landmark[INDEX_FINGER_TIP]
                index_loc = np.array([index_finger_tip.x, index_finger_tip.y])

                thumb_tip = hand_landmarks.landmark[THUMB_TIP]
                thumb_loc = np.array([thumb_tip.x, thumb_tip.y])

                distance = np.linalg.norm(index_loc - thumb_loc)

                if distance < DISTANCE_THRESHOLD:
                    midpoint = (index_loc + thumb_loc) / 2
                    curr_drawn_line.append(midpoint)

                else:
                    if len(curr_drawn_line) > 1:
                        drawn_lines.append(curr_drawn_line)
                        curr_drawn_line = []

        # draw a line between all points in drawn
        for drawn in drawn_lines:
            for i in range(1, len(drawn)):
                cv2.line(
                frame,
                (int(drawn[i - 1][0] * w), int(drawn[i - 1][1] * h)),
                (int(drawn[i][0] * w), int(drawn[i][1] * h)),
                (0, 255, 0),
                2,
                )

        if len(curr_drawn_line) > 1:
            for i in range(1, len(curr_drawn_line)):
                cv2.line(
                    frame,
                    (int(curr_drawn_line[i - 1][0] * w), int(curr_drawn_line[i - 1][1] * h)),
                    (int(curr_drawn_line[i][0] * w), int(curr_drawn_line[i][1] * h)),
                    (255, 0, 0),
                    2,
                )

        # mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow("frame", frame)
        out = cv2.waitKey(1)
        if out == ord("q"):
            break


if __name__ == "__main__":
    main()
