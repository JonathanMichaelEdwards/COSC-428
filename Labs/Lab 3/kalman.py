# kalman.py

import cv2
import numpy as np
from filterpy.kalman import KalmanFilter, UnscentedKalmanFilter, MerweScaledSigmaPoints

def find_ball(img, lower, upper):
    # Detect a colour ball with a colour range.
    mask = cv2.inRange(img, lower, upper)  # Find all pixels in the image within the colour range.

    # Find a series of points which outline the shape in the mask.
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        contour = max(contours, key=len)  # Assume the contour with the most points is the ball.
        # Fit a circle to the points of the contour enclosing the ball.
        (x,y),radius = cv2.minEnclosingCircle(contour)
        center = np.array([int(x),int(y)])
        radius = int(radius)
        return center, radius
    else:
        return None, None

def track_ball():
    # cap = cv2.VideoCapture('../images/red_ball_roll.avi')
    cap = cv2.VideoCapture('paper.avi')

    # Define the upper and lower colour thresholds for the ball colour.
    lower = np.array([30, 30, 90], dtype="uint8")
    upper = np.array([77, 87, 151], dtype="uint8")

    while cap.isOpened():
        ret, frame = cap.read()  # Read a frame from the video file.
        # If we cannot read any more frames from the video file, then exit.
        if not ret:
            break

        center, radius = find_ball(frame, lower, upper)
        if center is not None:
            # Draw circle around the ball.
            cv2.circle(frame, tuple(center), radius,(0,255,0), 2)
            # Draw the center (not centroid!) of the ball.
            cv2.circle(frame, tuple(center), 1,(0,255,0), 2)
        cv2.imshow('frame', frame)  # Display the grayscale frame on the screen.

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    # Release the video file, and close the GUI.
    cap.release()
    cv2.destroyAllWindows()

def track_ball_with_kalman():
    # cap = cv2.VideoCapture('../images/red_ball_roll.avi')
    cap = cv2.VideoCapture('images/red_ball_tunnel.avi')

    timestep = 1/25  # Time between frames in the video.

    # Construct the Kalman Filter and initialize the variables.
    kalman = KalmanFilter(dim_x=4, dim_z=2)  # 4 state variables, 2 measured variables.
    kalman.x = np.array([0, 0, 0, 0])  # Initial state for the filter.
    kalman.F = np.array([[1,0,timestep,0],
                         [0,1,0,timestep],
                         [0,0,1,0],
                         [0,0,0,1]], np.float32)  # State Transition Matrix
    kalman.H = np.array([[1,0,0,0],[0,1,0,0]], np.float32)  # Measurement matrix.
    kalman.P = np.array([[1000,0,0,0],
                         [0,1000,0,0],
                         [0,0,1000,0],
                         [0,0,0,1000]], np.float32)  # Covariance Matrix
    kalman.R = np.array([[1,0],
                         [0,1]], np.float32)  # Measurement Noise
    kalman.Q = np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,100,0],
                         [0,0,0,100]], np.float32)  # Process Noise

    # Define the upper and lower colour thresholds for the ball colour.
    lower = np.array([30,30,90], dtype="uint8")
    upper = np.array([77,87,151], dtype="uint8")

    while cap.isOpened():
        ret, frame = cap.read()  # Read an frame from the video file.
        if not ret: # If we cannot read any more frames from the video file, then exit.
            break

        center, radius = find_ball(frame, lower, upper)  # Search for the ball in the frame.
        print('\nMeasurement:\t', center)

        kalman.predict()  # Predict the ball's position.

        # Draw an ellipse showing the uncertainty of the predicted position.
        center_ = (int(kalman.x[0]), int(kalman.x[1]))
        axis_lengths = (int(kalman.P_prior[0, 0]), int(kalman.P_prior[1, 1]))
        cv2.ellipse(frame, center_, axis_lengths, 0, 0, 360, color=(255, 0, 0))

        if center is not None and radius is not None:
            cv2.circle(frame, tuple(center), radius, (0,255,0), 2)  # Draw circle around the ball.
            cv2.circle(frame, tuple(center), 1, (0,255,0), 2)  # Draw the center (not centroid!) of the ball.

            # The Kalman filter expects the x,y coordinates in a 2D array.
            measured = np.array([center[0], center[1]], dtype="float32")
            # Update the Kalman filter with the current ball location if we have it.
            kalman.update(measured)
            print('Estimate:\t', np.int32(kalman.x))
            print('Variance:\t', np.diag(kalman.P))

        cv2.imshow('frame', frame)  # Display the grayscale frame on the screen.

        if cv2.waitKey(80) & 0xFF == ord('q'):
            break

    # Release the video file, and close the GUI.
    cap.release()
    cv2.destroyAllWindows()

def track_ball_with_unscented_kalman():
    # cap = cv2.VideoCapture('../images/red_ball_roll.avi')
    cap = cv2.VideoCapture('images/red_ball_tunnel.avi')

    timestep = 1/25  # Time between frames in the video.

    # Construct the Kalman Filter and initialize the variables.
    def fx(x, dt):
        # State transition function.
        F = np.array([[1,0,dt,0],
                      [0,1,0,dt],
                      [0,0,1,0],
                      [0,0,0,1]], np.float32)
        return np.dot(F, x)

    def hx(x):
        # Extract the measurement from the state.
        return np.array(x[:2])
    
    points = MerweScaledSigmaPoints(4, 0.1, 2.0, 1)

    kalman = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=timestep, fx=fx, hx=hx, points=points)
    kalman.x = np.array([1,1,1,1])  # Initial State
    kalman.P = np.array([[2,0,0,0],
                         [0,2,0,0],
                         [0,0,2,0],
                         [0,0,0,2]], np.float32)  # Covariance Matrix
    kalman.R = np.array([[1,0],
                         [0,1]], np.float32)  # Measurement Noise
    kalman.Q = np.array([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,100,0],
                         [0,0,0,100]], np.float32)  # Process Noise

    # Define the upper and lower colour thresholds for the ball colour.
    lower = np.array([30,30,90], dtype="uint8")
    upper = np.array([77,87,151], dtype="uint8")

    while cap.isOpened():
        ret, frame = cap.read()  # Read an frame from the video file.
        if not ret: # If we cannot read any more frames from the video file, then exit.
            break

        center, radius = find_ball(frame, lower, upper)  # Search for the ball in the frame.
        print('\nMeasurement:\t', center)

        kalman.predict()  # Predict the ball's position.

        # Draw an ellipse showing the uncertainty of the predicted position.
        center_ = (int(kalman.x[0]), int(kalman.x[1]))
        axis_lengths = (int(kalman.P_prior[0, 0]), int(kalman.P_prior[1, 1]))
        cv2.ellipse(frame, center_, axis_lengths, 0, 0, 360, color=(255, 0, 0))

        if center is not None and radius is not None:
            cv2.circle(frame, tuple(center), radius, (0,255,0), 2)  # Draw circle around the ball.
            cv2.circle(frame, tuple(center), 1, (0,255,0), 2)  # Draw the center (not centroid!) of the ball.

            # The Kalman filter expects the x,y coordinates in a 2D array.
            measured = np.array([center[0], center[1]], dtype="float32")
            # Update the Kalman filter with the current ball location if we have it.
            kalman.update(measured)
            print('Estimate:\t', np.int32(kalman.x))
            print('Variance:\t', np.diag(kalman.P))

        cv2.imshow('frame', frame)  # Display the grayscale frame on the screen.

        if cv2.waitKey(80) & 0xFF == ord('q'):
            break

    # Release the video file, and close the GUI.
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    track_ball()
    # track_ball_with_kalman()
    # track_ball_with_unscented_kalman()
