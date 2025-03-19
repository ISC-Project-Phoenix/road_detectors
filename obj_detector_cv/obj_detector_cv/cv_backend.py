import cv2
import numpy as np
# import os
from tkinter import Tk  # this is for selecting files we dont need it 
from tkinter.filedialog import askopenfilename  # ^

# Function to open a file dialog for video selection
# we shouldnt be using this!
def open_file_dialog(initial_dir, file_types=[("MP4 files", "*.mp4"), ("All files", "*.*")]):
    """Opens a file dialog using Tkinter to select a video file.

    Args:
        initial_dir: The initial directory to open the dialog in.
        file_types: A list of tuples specifying the file types to filter.

    Returns:
        The selected file path, or an empty string if no file is selected.
    """
    root = Tk()  # Create a Tkinter root window
    root.withdraw()  # Hide the root window
    root.attributes('-topmost', True) # Bring the dialog to the front
    file_path = askopenfilename(initialdir=initial_dir, filetypes=file_types)  # Open the file dialog
    return file_path  # Return the selected file path

# Function to mask green color to black in an HSV frame
# nesscary
def mask_green_to_black(hsv_frame, lower_green, upper_green):
    """Masks green pixels in an HSV frame and replaces them with black.

    Args:
        hsv_frame: The input HSV frame.
        lower_green: The lower bound for the green color in HSV.
        upper_green: The upper bound for the green color in HSV.

    Returns:
        A tuple containing the masked frame and the green mask.
    """
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)  # Create a mask for green pixels
    result_frame = hsv_frame.copy()  # Copy the original frame to avoid modifying it directly
    black_hsv = np.array([0, 0, 0])  # Define black color in HSV
    result_frame[green_mask == 255] = black_hsv  # Replace green pixels with black
    return result_frame, green_mask  # Return the masked frame and the mask

# Function to fit a polynomial to a set of points
# very useful!
def fit_polynomial(points, order=3):
    """Fits a polynomial of a given order to a set of points.

    Args:
        points: A list of (y, x) tuples representing the points.
        order: The order of the polynomial to fit.

    Returns:
        A numpy.poly1d object representing the polynomial, or None if there are not enough points.
    """
    if len(points) < order + 1:  # Check if there are enough points to fit the polynomial
        return None  # Return None if not enough points
    x = [p[1] for p in points]  # Extract x-coordinates
    y = [p[0] for p in points]  # Extract y-coordinates
    coeffs = np.polyfit(y, x, order)  # Fit a polynomial to the points (y as function of x)
    polynomial = np.poly1d(coeffs)  # Create a polynomial object
    return polynomial  # Return the polynomial object

# Empty function used for trackbar callback
def nothing(x):
    """A dummy function used as a callback for trackbars."""
    pass  # Does nothing

# Function to process the video(s)
# TODO process return image!
def process_videos(frame):
    """Processes the video(s) to detect lane lines.

    Args:
        video_paths: A list of paths to the video files.
    """
    # TODO Rework for cv::Images!

    # Create a window for displaying the output stages
    cv2.namedWindow("Output Stages", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Output Stages", 1920, 720)  # Resize the window

    # Create trackbars for controlling the lower and upper bounds of the green color
    cv2.createTrackbar("Lower H", "Output Stages", 20, 179, nothing)
    cv2.createTrackbar("Upper H", "Output Stages", 52, 179, nothing)
    cv2.createTrackbar("Lower S", "Output Stages", 24, 255, nothing)
    cv2.createTrackbar("Upper S", "Output Stages", 255, 255, nothing)
    cv2.createTrackbar("Lower V", "Output Stages", 45, 255, nothing)
    cv2.createTrackbar("Upper V", "Output Stages", 255, 255, nothing)
    cv2.createTrackbar("GaussianBlur Ksize", "Output Stages", 13, 31, nothing)  # Odd values only
    cv2.createTrackbar("Canny Low Threshold", "Output Stages", 157, 255, nothing)
    cv2.createTrackbar("Canny High Threshold", "Output Stages", 43, 255, nothing)

    paused = False  # Initialize pause flag

    """
    TODO:
        remove while loop from scope. 
        remove for loop for video frames
        convert all frames into single use!
    """
    if True:  # Main loop for processing frames
        if true:  # Process frames only if not paused
            if true:  # Iterate through each video capture
                # resize frame!
                frame_resized = cv2.resize(frame, (640, 480))  # Resize the frame for processing


                # from function header
                frame_resized = cv2.resize(frame, (640, 480))  # Resize the frame for processing, must match the camera frame. 
                height, width = frame_resized.shape[:2]  # Get the height and width of the frame

                hsv_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)  # Convert the frame to HSV

                # Get trackbar positions for color thresholds
                lower_h = cv2.getTrackbarPos("Lower H", "Output Stages")
                upper_h = cv2.getTrackbarPos("Upper H", "Output Stages")
                lower_s = cv2.getTrackbarPos("Lower S", "Output Stages")
                upper_s = cv2.getTrackbarPos("Upper S", "Output Stages")
                lower_v = cv2.getTrackbarPos("Lower V", "Output Stages")
                upper_v = cv2.getTrackbarPos("Upper V", "Output Stages")
                gaussian_ksize = cv2.getTrackbarPos("GaussianBlur Ksize", "Output Stages")
                if gaussian_ksize % 2 == 0:
                    gaussian_ksize += 1
                canny_low = cv2.getTrackbarPos("Canny Low Threshold", "Output Stages")
                canny_high = cv2.getTrackbarPos("Canny High Threshold", "Output Stages")

                lower_green = np.array([lower_h, lower_s, lower_v])  # Define the lower green color bound
                upper_green = np.array([upper_h, upper_s, upper_v])  # Define the upper green color bound

                masked_frame, green_mask = mask_green_to_black(hsv_frame, lower_green, upper_green)  # Mask the green color

                result_bgr = cv2.cvtColor(masked_frame, cv2.COLOR_HSV2BGR)  # Convert the masked frame back to BGR
                green_bgr = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)  # Convert the mask to BGR

                # Create a mask to only process the lower part of the image
                mask_start = int(height * (52 / 100.0))  # Start masking from 48% of the image height
                mask = np.zeros_like(result_bgr)  # Create a mask of zeros
                mask[mask_start:, :] = 255  # Set the lower part of the mask to 255 (white)

                edge_mask_start = int(height * (52 / 100.0))
                edgemask = np.zeros_like(result_bgr)
                edgemask[edge_mask_start:, :] = 255

                roi_frame = cv2.bitwise_and(result_bgr, mask)  # Apply the mask to the masked frame
                roi_green = cv2.bitwise_and(green_bgr,mask)    # Apply the mask to rgb'ed green_mask

                #blurred_frame = cv2.GaussianBlur(roi_frame, (25, 25), 0)
                #edges = cv2.Canny(blurred_frame, 90, 180)
                blurred_frame = cv2.GaussianBlur(roi_green, (gaussian_ksize, gaussian_ksize),
                                                 0)  # Apply Gaussian blur to the ROI of the green mask
                edges = cv2.Canny(blurred_frame, canny_low,
                                  canny_high)  # Apply Canny edge detection to the blurred frame
                edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert the edges to BGR for display
                roi_edges = cv2.bitwise_and(edges_bgr, edgemask)  # Apply the edge mask.

                # 1. Morphological Closing to Join Broken Edges:
                kernel = np.ones((3, 3), np.uint8)  # Define a kernel for morphological operations (5x5 square)
                closed_edges = cv2.morphologyEx(roi_edges, cv2.MORPH_CLOSE,
                                                kernel)  # Perform morphological closing to close gaps in edges
                closed_edges_gray = cv2.cvtColor(closed_edges, cv2.COLOR_BGR2GRAY)  # Convert closed edges to grayscale

                # 2. Find Contours and Separate Left/Right:
                contours, _ = cv2.findContours(closed_edges_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contours_sorted = sorted(contours, key=lambda c: cv2.arcLength(c, True), reverse=True)
                longest_contours = contours_sorted[:2]  # Get the top 2 longest contours (for left and right)

                edges_largest_two = np.zeros_like(closed_edges)  # Keep this for visualization
                polynomial_frame = frame_resized.copy()

                left_contours = []
                right_contours = []

                for contour in longest_contours:
                    contour_points = np.array(contour).reshape(-1, 2)
                    avg_x = np.mean(contour_points[:, 0])  # Average x-coordinate of the contour

                    if avg_x < width // 2:
                        left_contours.append(contour)
                        cv2.drawContours(edges_largest_two, [contour], -1, (255, 0, 0), 3)  # Draw left contours in blue
                    else:
                        right_contours.append(contour)
                        cv2.drawContours(edges_largest_two, [contour], -1, (0, 0, 255), 3)  # Draw right contours in red

                # Process Left Contours:
                for contour in left_contours:
                    l_points = np.array(contour).reshape(-1, 2).tolist()  # Convert contour to list of points
                    long_l_points = [(y, x) for x, y in l_points]  # Correct order for polyfit

                    if long_l_points:
                        left_polynomial = fit_polynomial(long_l_points, order=3)
                        if left_polynomial:
                            # ... (rest of the polynomial fitting and drawing logic - same as before)
                            min_y_left = min(p[0] for p in long_l_points)
                            max_y_left = max(p[0] for p in long_l_points)
                            y_vals_left = np.linspace(min_y_left, max_y_left, 500)
                            x_vals_left = left_polynomial(y_vals_left)

                            valid_indices_left = (x_vals_left >= 0) & (x_vals_left < frame_resized.shape[1]) & (
                                    y_vals_left >= 0) & (y_vals_left < frame_resized.shape[0])
                            x_vals_left = x_vals_left[valid_indices_left]
                            y_vals_left = y_vals_left[valid_indices_left]

                            curve_points_left = np.array(list(zip(x_vals_left.astype(int), y_vals_left.astype(int))),
                                                         np.int32)
                            curve_points_left = curve_points_left.reshape((-1, 1, 2))

                            if curve_points_left.size > 0:
                                cv2.polylines(polynomial_frame, [curve_points_left], isClosed=False,
                                              color=(255, 255, 0),
                                              thickness=3)

                # Process Right Contours (same structure as left):
                for contour in right_contours:
                    r_points = np.array(contour).reshape(-1, 2).tolist()
                    long_r_points = [(y, x) for x, y in r_points]

                    if long_r_points:
                        right_polynomial = fit_polynomial(long_r_points, order=3)
                        if right_polynomial:
                            # ... (rest of the polynomial fitting and drawing logic - same as before)
                            min_y_right = min(p[0] for p in long_r_points)
                            max_y_right = max(p[0] for p in long_r_points)
                            y_vals_right = np.linspace(min_y_right, max_y_right, 500)
                            x_vals_right = right_polynomial(y_vals_right)

                            valid_indices_right = (x_vals_right >= 0) & (x_vals_right < frame_resized.shape[1]) & (
                                    y_vals_right >= 0) & (y_vals_right < frame_resized.shape[0])
                            x_vals_right = x_vals_right[valid_indices_right]
                            y_vals_right = y_vals_right[valid_indices_right]

                            curve_points_right = np.array(list(zip(x_vals_right.astype(int), y_vals_right.astype(int))),
                                                          np.int32)
                            curve_points_right = curve_points_right.reshape((-1, 1, 2))

                            if curve_points_right.size > 0:
                                cv2.polylines(polynomial_frame, [curve_points_right], isClosed=False,
                                              color=(255, 255, 0),
                                              thickness=3)


                cv2.imshow("Original Frame", frame_resized)
                cv2.imshow("Masked Frame", roi_frame)
                cv2.imshow("Edges", edges)  # Original Edges
                cv2.imshow("Longest Two Edges", edges_largest_two)  # Longest Two Edges
                cv2.imshow("Polynomial Fit", polynomial_frame)
                cv2.imshow("Green Mask", green_mask)
                cv2.imshow("Closed Edges", closed_edges)  # Edges after closing
                cv2.imshow("roi_edges", roi_edges)
        # end if frame is valid code 
"""
        # this appears to be switching video feeds?
        key = cv2.waitKey(25)

        if key == ord('q'):
            break
        elif key == ord(' '):
            paused = not paused
        elif key == ord('o'):
            print("Opening file selector...")
            # new_file_path = open_file_dialog(os.path.dirname(video_path))
            if new_file_path:
                print(f"Switching to new file: {new_file_path}")
                caps[0].release()
                caps = [cv2.VideoCapture(new_file_path)]
                video_paths = os.path.basename(new_file_path).lower()

        elif key == 81:  # Left arrow key (rewind)
            current_frame = caps[0].get(cv2.CAP_PROP_POS_FRAMES)
            caps[0].set(cv2.CAP_PROP_POS_FRAMES, max(0, current_frame - 25))
        elif key == 83:  # Right arrow key (fast forward)
            current_frame = caps[0].get(cv2.CAP_PROP_POS_FRAMES)
            caps[0].set(cv2.CAP_PROP_POS_FRAMES, min(caps[0].get(cv2.CAP_PROP_FRAME_COUNT) - 1, current_frame + 25))
        elif key == ord('j'):  # Jump backward
            current_frame = caps[0].get(cv2.CAP_PROP_POS_FRAMES)
            caps[0].set(cv2.CAP_PROP_POS_FRAMES, max(0, current_frame - 100))
        elif key == ord('l'):  # Jump forward
            current_frame = caps[0].get(cv2.CAP_PROP_POS_FRAMES)
            caps[0].set(cv2.CAP_PROP_POS_FRAMES, min(caps[0].get(cv2.CAP_PROP_FRAME_COUNT) - 1, current_frame + 100))
    # end while true loop

    for cap in caps:
        cap.release()
    cv2.destroyAllWindows()

directory = "/home/derek/Downloads"  # Replace with your default directory
video_paths = []

print("Select a video file")
video_path = open_file_dialog(directory)
if video_path:
    video_paths.append(video_path)
    process_videos(video_paths)
else:
    print("No video selected.")
"""