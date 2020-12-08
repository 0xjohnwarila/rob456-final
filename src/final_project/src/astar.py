import numpy as np
import matplotlib.pyplot as plt
import mapreading  
import heapq as hq

def astar(m, target, start):

    # Setup data structures

    # priority queue
    q = []

    closed = set()
    # points and their priority (dist from the goal)
    points = {}

    parents = {}

    #
    path = []
    # put the first node in the queue
    hq.heappush(q, (0, start))
    points[start] = 0

    i = 0
    while q:
        i += 1

        # get closest to target point
        curr = hq.heappop(q)
        closed.add(curr[1])

        if curr[1] == target:
            return points, parents, i

        # get neighbors
        neighbors = mapreading.neighbors_explored(m, curr[1])
        for n in neighbors:
            neighbor = (n[0], n[1])
            if neighbor in closed:
                continue
            
            # calc distance to target
            new_priority = abs(neighbor[0] - target[0]) + abs(neighbor[1] - target[1])

            # check if we have already been here
            if points.has_key(neighbor):
                # update priority
                if points[neighbor] > new_priority:
                    points[neighbor] = new_priority
            else:
                # add new node
                points[neighbor] = new_priority
                hq.heappush(q, (new_priority, neighbor))
                parents[neighbor] = curr[1]

    return points, parents, i


if __name__ == '__main__':
    img = mapreading.threshold('murrmatt.pgm')

    new_img, parents, dist = astar(img, (115, 90), (174, 188))
    
    print dist

    y = np.size(img, axis=0)
    x = np.size(img, axis=1)
    img2 = np.full([y, x], np.inf)


    child = (115, 90)
    target  = (174, 188)
    path = []
    i = 0
    while child != target:
        if i % 5 == 0 or i == 0:
            path.append(child)
            img2[child[0]][child[1]] = new_img[child]
        child = parents[child]
        i += 1

    print path
    plt.imshow(img)
    plt.imshow(img2)

    plt.show()

