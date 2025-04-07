import cv2
import numpy as np

# Function to open a file dialog for video selection
# we shouldnt be using this!

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
def fit_polynomial(points, order=2):
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

    # Create a window for displaying the Processing Stages
    cv2.namedWindow("Processing Stages", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("Processing Stages", 1920, 720)  # Resize the window


    # Create trackbars for controlling the lower and upper bounds of the green color
    cv2.createTrackbar("ROI", "Processing Stages", 52,256,nothing)
    cv2.createTrackbar("Lower H", "Processing Stages", 20, 179, nothing)
    cv2.createTrackbar("Upper H", "Processing Stages", 52, 179, nothing)
    cv2.createTrackbar("Lower S", "Processing Stages", 35, 255, nothing) #24
    cv2.createTrackbar("Upper S", "Processing Stages", 255, 255, nothing)
    cv2.createTrackbar("Lower V", "Processing Stages", 45, 255, nothing)
    cv2.createTrackbar("Upper V", "Processing Stages", 255, 255, nothing)
    cv2.createTrackbar("GaussianBlur Ksize", "Processing Stages", 13, 31, nothing)  # Odd values only
    cv2.createTrackbar("Canny Low Threshold", "Processing Stages", 157, 255, nothing)
    cv2.createTrackbar("Canny High Threshold", "Processing Stages", 43, 255, nothing)

    paused = False  # Initialize pause flag

    """
    TODO:
        remove while loop from scope. 
        remove for loop for video frames
        convert all frames into single use!
    """

    # resize frame!
    # frame_resized = cv2.resize(frame, (640, 480))  # Resize the frame for processing
    # height, width = frame_resized.shape[:2]  # Get the height and width of the frame

    # DO NOT THE FRAME
    frame_resized = frame
    height, width = frame_resized.shape[:2]  # Get the height and width of the frame


    hsv_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2HSV)  # Convert the frame to HSV

    # Get trackbar positions for color thresholds
    roi_shade = cv2.getTrackbarPos("ROI", "Processing Stages")
    lower_h = cv2.getTrackbarPos("Lower H", "Processing Stages")
    upper_h = cv2.getTrackbarPos("Upper H", "Processing Stages")
    lower_s = cv2.getTrackbarPos("Lower S", "Processing Stages")
    upper_s = cv2.getTrackbarPos("Upper S", "Processing Stages")
    lower_v = cv2.getTrackbarPos("Lower V", "Processing Stages")
    upper_v = cv2.getTrackbarPos("Upper V", "Processing Stages")
    gaussian_ksize = cv2.getTrackbarPos("GaussianBlur Ksize", "Processing Stages")
    if gaussian_ksize % 2 == 0:
        gaussian_ksize += 1
    canny_low = cv2.getTrackbarPos("Canny Low Threshold", "Processing Stages")
    canny_high = cv2.getTrackbarPos("Canny High Threshold", "Processing Stages")

    lower_green = np.array([lower_h, lower_s, lower_v])  # Define the lower green color bound
    upper_green = np.array([upper_h, upper_s, upper_v])  # Define the upper green color bound

    masked_frame, green_mask = mask_green_to_black(hsv_frame, lower_green,
                                                    upper_green)  # Mask the green color

    result_bgr = cv2.cvtColor(masked_frame, cv2.COLOR_HSV2BGR)  # Convert the masked frame back to BGR
    green_bgr = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)  # Convert the mask to BGR

    # Create a mask to only process the lower part of the image
    mask_start = int(height * (roi_shade / 100.0))  # Start masking from 48% of the image height
    mask = np.zeros_like(result_bgr)  # Create a mask of zeros
    mask[mask_start:, :] = 255  # Set the lower part of the mask to 255 (white)

    edge_mask_start = int(height * (roi_shade / 100.0))
    edgemask = np.zeros_like(result_bgr)
    edgemask[edge_mask_start:, :] = 255

    # Create the bumper mask (INVERSE)
    bumper_mask_size = 100  # Adjust the size as needed
    bumper_mask = np.ones_like(result_bgr, dtype=np.uint8) * 255  # initialize to white
    bumper_mask_x = width // 2 - bumper_mask_size // 2
    bumper_mask_y = height - bumper_mask_size
    bumper_mask[bumper_mask_y:bumper_mask_y + bumper_mask_size,
    bumper_mask_x:bumper_mask_x + bumper_mask_size] = 0  # set the bumper part to black.

    #roi_frame = cv2.bitwise_and(result_bgr, mask)  # Apply the mask to the masked frame
    roi_green = cv2.bitwise_and(green_bgr, mask)  # Apply the mask to rgb'ed green_mask
    roi = cv2.bitwise_and(roi_green, bumper_mask)

    # blurred_frame = cv2.GaussianBlur(roi_frame, (25, 25), 0)
    # edges = cv2.Canny(blurred_frame, 90, 180)
    blurred_frame = cv2.GaussianBlur(roi, (gaussian_ksize, gaussian_ksize),
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

    edges_largest_two = np.zeros_like(closed_edges)  # Keep this for visualization
    polynomial_frame = frame_resized.copy()

    left_contours = []
    right_contours = []

    long_r_points = []
    long_l_points = []

    for contour in contours_sorted:
        contour_points = np.array(contour).reshape(-1, 2)
        avg_x = np.mean(contour_points[:, 0])  # Average x-coordinate of the contour

        if avg_x < width // 2:
            left_contours.append(contour)
            
        else:
            right_contours.append(contour)
            
    # Find the longest left and right contours:
    longest_left_contours = sorted(left_contours, key=lambda c: cv2.arcLength(c, True),
                                        reverse=True)[:1] if left_contours else []
    longest_right_contours = sorted(right_contours, key=lambda c: cv2.arcLength(c, True),
                                        reverse=True)[:1] if right_contours else []
    mininium_threshold = height * 0.35;

    perimeter = cv2.arcLength(longest_right_contours[0], True)
    if perimeter < mininium_threshold:
        # self.get_logger().info("right mininium threshold not meet.")
        return
    
    perimeter = cv2.arcLength(longest_right_contours[0], True)
    if perimeter < mininium_threshold:
        # self.get_logger().info("right mininium threshold not meet.")
        return

    # Process Left Contours:
    for contour in longest_left_contours:
        cv2.drawContours(edges_largest_two, [contour], -1, (255, 0, 0), 3)  # Draw left contours in blue
        l_points = np.array(contour).reshape(-1, 2).tolist()  # Convert contour to list of points
        long_l_points = [(y, x) for x, y in l_points]  # Correct order for polyfit

        if long_l_points:
            left_polynomial = fit_polynomial(long_l_points, order=2)
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
    for contour in longest_right_contours:
        cv2.drawContours(edges_largest_two, [contour], -1, (0, 0, 255), 3)  # Draw right contours in red
        r_points = np.array(contour).reshape(-1, 2).tolist()
        long_r_points = [(y, x) for x, y in r_points]

        if long_r_points:
            right_polynomial = fit_polynomial(long_r_points, order=2)
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

    # Get coefficients or fallback to zeros
    # for order change np.zeros() to the correct array length!
    left_coeffs = left_polynomial.coeffs if 'left_polynomial' in locals() and left_polynomial is not None else np.zeros(3)
    right_coeffs = right_polynomial.coeffs if 'right_polynomial' in locals() and right_polynomial is not None else np.zeros(3)
    


    if (np.any(left_coeffs) and np.any(right_coeffs)):
        average_coeffs = (left_coeffs + right_coeffs) / 2.0
        
        # Determine which contour to use based on arc length
        if longest_right_contours and longest_left_contours:
            right_length = cv2.arcLength(longest_right_contours[0], False)
            left_length = cv2.arcLength(longest_left_contours[0], False)
            longest_contour = longest_right_contours[0] if right_length > left_length else longest_left_contours[0]
        elif longest_right_contours:
            longest_contour = longest_right_contours[0]
        elif longest_left_contours:
            longest_contour = longest_left_contours[0]
        else:
            longest_contour = None

        if longest_contour is not None:
            # Convert contour to numpy array and reshape
            contour_points = np.array(longest_contour).reshape(-1, 2)
            
            contour_points = np.array(contour_points).reshape(-1, 2)
            y_points = contour_points[:, 1]  # Using y-values as input
            
            # Evaluate x = f(y) using the polynomial
            x_points = np.polyval(average_coeffs, y_points)
            
            # Combine and format for OpenCV
            curve_points_middle = np.column_stack((x_points, y_points)).astype(np.int32)
            curve_points_middle = curve_points_middle.reshape((-1, 1, 2))
            
            # Draw debug points (red circles) to verify positions
            for point in curve_points_middle[:,0,:]:
                cv2.circle(polynomial_frame, tuple(point), 3, (0,0,255), -1)
            
            # Draw the curve if we have valid points
            if curve_points_middle.size > 0:
                cv2.polylines(
                    polynomial_frame,
                    [curve_points_middle],
                    isClosed=False,
                    color=(255, 255, 155),  # Yellow
                    thickness=3
                )


    combined_frame = np.hstack([
        frame_resized,
        cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR),  # Convert green_mask to BGR
        closed_edges,
        edges_largest_two,
        polynomial_frame
    ])

    cv2.imshow("Processing Stages", combined_frame)
    cv2.imshow("Output", polynomial_frame)
    cv2.imshow("Original Frame", frame_resized)

    cv2.waitKey(1)
    return {
        "left_contours": left_contours,
        "right_contours": right_contours,
        "left_coeffs": left_coeffs,
        "right_coeffs": right_coeffs
    }
