import numpy as np
import matplotlib.pyplot as plt
import mapreading  
import heapq as hq

def get_neighbors(m, target):
    """
    param: m - map
    param: target - (y, x)
    return: array of neighbor points
    """
    ret = []

    ys = np.arange(target[0] - 1, target[0] + 2)
    xs = np.arange(target[1] - 1, target[1] + 2)
    
    for y in ys:
        for x in xs:
            if (y, x) == target:
                continue
            if m[y, x] > 1:
                ret.append((y, x))
    return ret

def floodfill(m, target):
    """
    param: m - map
    param: target - (y, x)
    return: flood filled map
    """
    q = []
    points = {}
    hq.heappush(q, (0, target))
    points[target] = 0

    while q:
        curr = hq.heappop(q)
        neighbors = mapreading.neighbors_explored(m, curr[1])
        for n in neighbors:
            neighbor = (n[0], n[1])
            new_priority = curr[0] + 1
            if points.has_key(neighbor):
                if points[neighbor] > new_priority:
                    points[neighbor] = new_priority
            else:
                points[neighbor] = new_priority
                hq.heappush(q, (new_priority, neighbor))

    return points


if __name__ == '__main__':
    img = mapreading.threshold('murrmatt.pgm')

    new_img = floodfill(img, (115, 90))
    print new_img
    plt.imshow(img)
    plt.show()
