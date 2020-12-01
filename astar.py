#!/usr/bin/env python2
import heapq

# queue
q = []

# (weight, edge)
heapq.heappush(q, (2, 'pt1'))
heapq.heappush(q, (1, 'pt2'))
heapq.heappush(q, (3, 'pt3'))
heapq.heappush(q, (10, 'pt4'))
heapq.heappush(q, (2.4, 'pt5'))

while q:
    next_item = heapq.heappop(q)
    print next_item
