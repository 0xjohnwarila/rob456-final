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


def neighbors_explored(map_array, coordinates):
    """
    Takes in a thresholded map array and a set of pixel coordinates and returns a list of coordinates mapping to the
    unoccupied areas in the 8 pixels surrounding the query point
    :param coordinates: numpy array of coordinates in (y, x) form (because numpy likes to reverse axes for some reason)
    :return: list of y, x coordinates representing unoccupied pixels
    """
    unoccupied = np.empty((0, 2), int)
    # Dumb set of if-else conditions to account for all four corners
    if coordinates[0] == 0 and coordinates[1] == 0:
        y = np.arange(coordinates[0], coordinates[0] + 2)
        x = np.arange(coordinates[1], coordinates[1] + 2)
    elif coordinates[0] == np.size(map_array, axis=0) - 1 and coordinates[1] == 0:
        y = np.arange(coordinates[0] - 1, coordinates[0] + 1)
        x = np.arange(coordinates[1], coordinates[1] + 2)
    elif coordinates[0] == np.size(map_array, axis=0) - 1 and coordinates[1] == np.size(map_array, axis=1) - 1:
        y = np.arange(coordinates[0] - 1, coordinates[0] + 1)
        x = np.arange(coordinates[1] - 1, coordinates[1] + 1)
    elif coordinates[0] == 0 and coordinates[1] == np.size(map_array, axis=1) - 1:
        y = np.arange(coordinates[0], coordinates[0] + 2)
        x = np.arange(coordinates[1] - 1, coordinates[1] + 1)
    else:
        y = np.arange(coordinates[0] - 1, coordinates[0] + 2)
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
    plt.imshow(img)
    plt.show()