import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt


def threshold(image):
    """
    Takes in slam image files and thresholds them so that walls = 0, explored areas = 255, and unexplored areas = inf
    :param image: Image file
    :return: Thresholded numpy array
    """
    img = mpimg.imread(image)
    y = np.size(img, axis=0)
    x = np.size(img, axis=1)
    new_img = np.full([y, x], np.inf)
    new_img[img == 0] = 0
    new_img[img == 254] = 255
    return new_img

def live_threshold(data):
    y = np.size(data, axis=0)
    x = np.size(data, axis=1)
    new_img = np.full([y, x], np.inf)
    new_img[data < 30] = 255
    new_img[data == -1] = 0
    return new_img
    


def neighbors_explored(map_array, coordinates):
    """
    Takes in a thresholded map array and a set of pixel coordinates and returns a list of coordinates mapping to the
    unoccupied areas in the 8 pixels surrounding the query point
    :param coordinates: numpy array of coordinates in (y, x) form (because numpy likes to reverse axes for some reason)
    :return: list of y, x coordinates representing unoccupied pixels
    """
    unoccupied = np.empty((0, 2), int)
    # Dumb set of if-else conditions to account for all four corners
    if coordinates[0] + 2 > np.size(map_array, axis=0) - 1:
        y = np.arange(coordinates[0] - 1, coordinates[0] + 1)
    elif coordinates[0] - 1 < 0:
        y = np.arange(coordinates[0], coordinates[0] + 2)
    else:
        y = np.arange(coordinates[0] - 1, coordinates[0] + 2)

    if coordinates[1] + 2 > np.size(map_array, axis=1) - 1:
        x = np.arange(coordinates[1] - 1, coordinates[1] + 1)
    elif coordinates[1] - 1 < 0:
        x = np.arange(coordinates[1], coordinates[1] + 2)
    else:
        x = np.arange(coordinates[1] - 1, coordinates[1] + 2)
    # Double for loop to iterate through all the pixels in the area of interest
    for num in y:
        for num2 in x:
            if num == coordinates[0] and num2 == coordinates[1]:
                continue
            else:
                if map_array[num, num2] > 1:
                    unoccupied = np.append(unoccupied, np.array([[num, num2]]), axis=0)
    return unoccupied


def get_boundary_pixels(map_array):
    """
    This runs through the map, finds unoccupied pixels that have three or more unexplored neighbors, then returns a list
    containing those boundary pixels (PS: I hate the way I wrote this function)
    :param map_array: Thresholded numpy array representing SLAM map
    :return: list of boundary pixels
    """
    pixel_list = np.empty((0, 2), int)
    for r in np.arange(np.size(map_array, axis=0)):
        for c in np.arange(np.size(map_array, axis=1)):
            if map_array[r, c] == 255:
                if r + 2 > np.size(map_array, axis=0) - 1:
                    y = np.arange(r - 1, r + 1)
                elif r - 1 < 0:
                    y = np.arange(r, r + 2)
                else:
                    y = np.arange(r - 1, r + 2)
                if c + 2 > np.size(map_array, axis=1) - 1:
                    x = np.arange(c - 1, c + 1)
                elif c - 1 < 0:
                    x = np.arange(c, c + 2)
                else:
                    x = np.arange(c - 1, c + 2)
                unexplored = 0
                for num in y:
                    for num2 in x:
                        if map_array[num, num2] > 255:
                            unexplored += 1
                if unexplored > 2:
                    pixel_list = np.append(pixel_list, np.array([[r, c]]), axis=0)
    return pixel_list


if __name__ == '__main__':
    img = threshold('murrmatt.pgm')
    test_points = np.array([[0, 0],
                           [0, 383],
                           [383, 383],
                           [77, 163],
                           [383, 0]])
    test1 = neighbors_explored(img, test_points[0, :])
    test2 = neighbors_explored(img, test_points[1, :])
    test3 = neighbors_explored(img, test_points[2, :])
    test4 = neighbors_explored(img, test_points[3, :])
    test5 = neighbors_explored(img, test_points[4, :])
    print test1
    print test2
    print test3
    print test4
    plt.imshow(img)
    plt.show()