import numpy as np
import matplotlib.pyplot as plt
import mapreading  
import heapq as hq

def floodfill(m, target):
    # Setup data structures

    # priority queue
    q = []
    
    # points and their priority
    points = {}
    closed = set()

    # put the first node in the queue
    hq.heappush(q, (0, target))
    points[target] = 0

    while q:
        
        # get lowest priority (but really high for the algo)
        curr = hq.heappop(q)
        closed.add(curr[1])
        # get neighbors
        neighbors = mapreading.neighbors_explored(m, curr[1])
        for n in neighbors:
            neighbor = (n[0], n[1])
            if neighbor in closed:
                continue
            # new priority
            new_priority = curr[0] + 1
            if points.has_key(neighbor):
                # update priority
                if points[neighbor] > new_priority:
                    points[neighbor] = new_priority
            else:
                # add new node
                points[neighbor] = new_priority
                hq.heappush(q, (new_priority, neighbor))

    return points


if __name__ == '__main__':
    img = mapreading.threshold('murrmatt.pgm')

    new_img = floodfill(img, (115, 90))

    y = np.size(img, axis=0)
    x = np.size(img, axis=1)
    img = np.full([y, x], np.inf)
    for pixel in new_img:
        img[pixel[0]][pixel[1]] = new_img[pixel]

    plt.imshow(img)
    plt.show()

