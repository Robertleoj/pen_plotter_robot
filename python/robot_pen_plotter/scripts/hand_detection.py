import cv2
import numpy as np
import math
import mediapipe as mp
from plotter import Plotter

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils


INDEX_FINGER_TIP = 8
THUMB_TIP = 4

DISTANCE_THRESHOLD = 0.05

drawn_lines: list[list[np.ndarray]] = []
curr_drawn_line = []

def draw_hand(frame: np.ndarray, hand_landmarks: mp.solutions.hands.HandLandmark) -> None:

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


def main():
    global drawn_lines
    global curr_drawn_line

    plotter = Plotter(
        serial_port="/dev/ttyACM0",
        x_input_range=(0, 1),
        y_input_range=(0, 1),
    )

    cap = cv2.VideoCapture(0)
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
    )

    while True:
        ret, frame = cap.read()
        h, w, _ = frame.shape
        # ummirror
        frame = cv2.flip(frame, 1)

        if not ret:
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)

        if results.multi_hand_landmarks is not None and len(results.multi_hand_landmarks) == 2:
            hand_1_coords = np.array([
                [lm.x, lm.y]
                for lm in results.multi_hand_landmarks[0].landmark
            ])

            hand_2_coords = np.array([
                [lm.x, lm.y]
                for lm in results.multi_hand_landmarks[1].landmark
            ])

            # hand with lower x-coordinate is the left hand
            if hand_1_coords[:, 0].mean() < hand_2_coords[:, 0].mean():
                # left_hand_idx, right_hand_idx = 0, 1
                left_hand_coords, right_hand_coords = hand_1_coords, hand_2_coords
            else:
                # left_hand_idx, right_hand_idx = 1, 0
                left_hand_coords, right_hand_coords = hand_2_coords, hand_1_coords

            # left_hand_landmarks = results.multi_hand_landmarks[left_hand_idx]
            # right_hand_landmarks = results.multi_hand_landmarks[right_hand_idx]

            # draw_hand(frame, left_hand_landmarks)
            # draw_hand(frame, right_hand_landmarks)



            # check if the left hand is pinching

            left_index_loc = left_hand_coords[INDEX_FINGER_TIP]
            left_thumb_loc = left_hand_coords[THUMB_TIP]
            right_index_loc = right_hand_coords[INDEX_FINGER_TIP]

            right_left_index_distance = np.linalg.norm(right_index_loc - left_index_loc)

            if right_left_index_distance < DISTANCE_THRESHOLD:
                plotter.home()
                break

            left_thumb_index_distance = np.linalg.norm(left_index_loc - left_thumb_loc)

            if left_thumb_index_distance < DISTANCE_THRESHOLD:
                right_index_loc = right_hand_coords[INDEX_FINGER_TIP]

                curr_drawn_line.append(right_index_loc)

                plotter.move_to(np.array([
                    right_index_loc[0],
                    1 - right_index_loc[1],
                ]))

                if len(curr_drawn_line) == 1:
                    plotter.pendown()

            else:
                if len(curr_drawn_line) >= 1:
                    drawn_lines.append(curr_drawn_line)
                    curr_drawn_line = []
                    plotter.penup()

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

    plotter.wait_until_done()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
