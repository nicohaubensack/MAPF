import math
import random
import Util

class MAPF_Inst:
    def __init__(self,V,S,D,E,CellSize,GridDimension,obstacles_static = [[],[]], obstacles_moving = [[]],imagePath = "",trees = [],person = None,road = []):
        self.V = V
        self.S = S
        self.D = D
        self.E = E
        self.CellSize = CellSize
        self.GridDimension = GridDimension
        self.imagePath = imagePath
        self.obstacles = Util.get_obstacle_nodes(obstacles_static,obstacles_moving)
        self.trees = trees
        self.person = person
        self.road = road

inst1 = MAPF_Inst([(0,0),(0,2),(1,1),(2,0),(2,2)],
                  [[0,1]],
                  [[3,4]],
                  [[2],[2],[0,1,3,4],[2],[2]],
                  100,
                  (3,3))
inst2 = MAPF_Inst([(1,0),(2,0),(0,1),(1,1),(2,1),(3,1),(4,1),(0,2),(1,2),(2,2),(3,2),(4,2),(1,3),(2,3),(3,3),(4,3),(1,4),(2,4),(3,4),(4,4)],
                  [[5,6,16],[18,19]],
                  [[0,2,3],[7,8]],
                  [[1,3],[0,4],[3,7],[0,2,4,8],[1,3,9],[10],[11],[2],[3],[4,10,13],[5,9,11],[6,10,15],[8,13,16],[9,12,17],[15,18],[11,14,19],[12],[13,18],[14,17],[15]],
                  100,
                  (5,5),
                  obstacles_static=[[(9,2),(9,4)],[]])
inst3 = MAPF_Inst([(1,0),(3,0),(4,0),(0,1),(1,1),(3,1),(4,1),(1,2),(2,2),(3,2),(4,2)],
                       [[0,3,7]],
                       [[1,2,6]],
                       [[4],[5],[5],[4],[0,3,5,7],[1,2,4,6],[5],[4,8],[7,9],[8,10],[6,9]],
                       100,
                       (5,3))

def partition_agents_to_groups(S_list,D_list,groups,keep_order):
    num_agents = len(S_list)
    amounts = [1 for _ in range(groups)]
    for _ in range(num_agents - groups):
        r = math.floor(random.random() * groups)
        amounts[r] += 1
    

    if keep_order == False:
        random.shuffle(S_list)
        random.shuffle(D_list)
    S = [[] for _ in range(groups)]
    D = [[] for _ in range(groups)]

    
    already = 0
    for g in range(groups):
        S[g] = S_list[already : already + amounts[g]]
        D[g] = D_list[already : already + amounts[g]]
        already += amounts[g]
        
    return S, D

def generateDeliveryInstance(vertical_roads,horizontal_roads,agents,groups,seed = None):
    if seed is not None:
        random.seed(seed)
    groups = min(4,groups)
    
    S = list(range(agents))
    D = list(range(agents))
    S,D = partition_agents_to_groups(S,D,groups,True)
    width = max([math.ceil(math.sqrt(len(S[i]))) for i in range(len(S))])
    puf = max(2,width)
    w = 1+2*puf +3*(vertical_roads-1)
    h = 1+2*puf +3*(horizontal_roads-1)

    V = []
    
    for i in range(vertical_roads):
        V += [(puf+3*i,y) for y in range(h)]
    for i in range(horizontal_roads):
        V += [(x,puf+3*i) for x in range(w) if (x,puf+3*i) not in V]

    agents = min(len([1 for x in range(puf+1,w-puf-1) for y in range(puf,h-puf ) if (x-puf)%3 == 0 and (y-puf)%3 == 0] ),agents)
    random.shuffle(V)
    idx = 0
    for i in range(groups):
        for j in range(len(S[i])):
            x = width - 1- (j - width * math.floor(j / width)) if i in [0,2] else (j - width * math.floor(j / width))
            y = math.floor(j / width) if i < 2 else width -1- math.floor(j / width)
            if i == 0:
                pos = (x,h-puf+y)
            elif i == 1:
                pos = (w-puf+x,h-puf+y)
            elif i == 2:
                pos = (x,y)
            elif i == 3:
                pos = (w-puf+x,y)
            V += [pos]
            S[i][j] = len(V)-1

            while True:
                x = random.randint(puf+1,w-puf-1)
                y = random.randint(puf+1,h-puf-1)
                if (x-puf)%3 == 0 or (y-puf)%3 == 0 or (x,y) in V:
                    continue
                V += [(x,y)]
                D[i][j] = len(V) -1
                break
    dest_set = set()
    for group in D:
        for idx in group:
            dest_set.add(idx)

    E = [[] for _ in range(len(V))]
    coord_to_idx = {coords: idx for idx, coords in enumerate(V)}

    for i, (x, y) in enumerate(V):
        if i in dest_set:
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (x + dx, y + dy)
                if neighbor in coord_to_idx and (x + dx - puf) % 3 == 0 or (y + dy - puf) % 3 == 0:
                    neighbor_idx = coord_to_idx[neighbor]
                    E[i].append(neighbor_idx)
                    E[neighbor_idx].append(i)
                    break
        else:
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (x + dx, y + dy)
                if neighbor in coord_to_idx:
                    neighbor_idx = coord_to_idx[neighbor]
                    if neighbor_idx not in dest_set and neighbor_idx > i:
                        E[i].append(neighbor_idx)
                        E[neighbor_idx].append(i)  

    return MAPF_Inst(V, S, D, E, 100, (w, h))      
    
def generateGameInstance(agent_spawn_width,agents,trees = 5,seed = None,cars = True):
    if seed is not None:
        random.seed(seed)
    h = 7
    w = 8+agent_spawn_width
    V = [(i,j) for j in range(h) for i in range(w)]
    n = len(V)
    E = [[] for _ in range(n)]
    for i in range(n):
        E[i] += [i-1] if i%w != 0 else []
        E[i] += [i+1] if (i+1)%w != 0 else []
        E[i] += [i+w] if i < w*(h-1) else []
        E[i] += [i-w] if i >= w else []

    road = [w-5,w-4]

    D = [[w*(3+j)-3 + i for i in range(3) for j in range(3) if (i,j) != (1,1)]]
    
    possible = [w*j + i for i in range(agent_spawn_width) for j in range(h)]
    random.shuffle(possible)
    S = [possible[:agents]]

    obstacles = []
    t1,t2 = -10,-10
    if cars:
        for i in range(100):
            wait = random.randint(2,4)
            t1 += wait
            obstacles += [[(road[1]+w*(t-t1),t) for t in range(t1,t1+h)]]

            wait = random.randint(2,4)
            t2 += wait
            obstacles += [[(road[0]+w*(h-1)-w*(t-t2),t) for t in range(t2,t2+h)]]

    tree_positions = [i+w*j for j in range(h) for i in range(w) if i not in [w-5, w-4] and i+w*j not in S[0] and i+w*j not in D[0] and (i, j) != (w-2, 3)]
    random.shuffle(tree_positions)
    trees_added = []
    person = None
    x = -1
    while len(trees_added) < trees:
        if x == -1: pos = 3*w + w-2
        else: pos = tree_positions[x]
        E_cop = [[i if i < pos else i-1 for i in e if i != pos] for idx, e in enumerate(E) if idx != pos]
        
        if Util.is_connected(E_cop):
            if x == -1: person = V.pop(pos)
            else: trees_added += [V.pop(pos)]
            E = E_cop
            S = [[i if i < pos else i-1 for i in group] for group in S]
            D = [[i if i < pos else i-1 for i in group] for group in D]
            obstacles = [[(i,t) if i < pos else (i-1,t) for (i,t) in car] for car in obstacles]
            tree_positions = [i if i < pos else i-1 for i in tree_positions]
            
        x += 1
    

    return MAPF_Inst(V, S, D, E, 100, (w, h), obstacles_moving=obstacles,trees = trees_added,person=person,road=road)

def generateGridHorizontalPaths(w,h,groups,keep_edges_percent = 1,trucks = 0,keep_order = False, seed = None,buffer = 0):
    return generateGrid(w,h,h,groups,agentsLeft = True,keep_edges_percent=keep_edges_percent, trucks=trucks,keep_order = keep_order,seed = seed,buffer = buffer)

def generateRandomGeometric(num_nodes, radius, agents, groups, seed=None,it = 0):
    if seed is not None:
        random.seed(seed)
    
    V = [(random.random(), random.random()) for _ in range(num_nodes)]
    E = [[] for _ in range(num_nodes)]
    for i, (x1, y1) in enumerate(V):
        for i2, (x2, y2) in enumerate(V[i+1:]):
            d = math.sqrt((x1-x2)**2 + (y1-y2)**2)
            if d <= radius:
                E[i] += [i+1+i2]
                E[i+1+i2] += [i]

    if not Util.is_connected(E):
        if it > 400: 
            print("could not find connected graph, consider increasing radius")
            return None
        return generateRandomGeometric(num_nodes, radius, agents, groups, seed=random.randint(0, 10000000),it = it+1)
    
    indices = list(range(num_nodes))
    random.shuffle(indices)
    
    S_list = indices[0:agents]
    D_list = indices[agents:2*agents]

    S, D = partition_agents_to_groups(S_list, D_list, groups, keep_order=False)
    V = [(10*x,10*y) for (x,y) in V] # make it visually better
    return MAPF_Inst(V, S, D, E, 100, (10, 10))

def generateGrid(w,h,agents,groups,agentsLeft = False, delete_edges = 0, keep_edges_percent = 1, trucks = 0, seed = None, keep_order = False,buffer = 0):
    if seed != None: random.seed(seed)
    w = w+ 2* buffer
    h = h+2*buffer
    V = [(i,j) for j in range(h) for i in range(w)]
    n = len(V)
    E = [[] for _ in range(n)]
    for i in range(n):
        E[i] += [i-1] if i%w != 0 else []
        E[i] += [i+1] if (i+1)%w != 0 else []
        E[i] += [i+w] if i < w*(h-1) else []
        E[i] += [i-w] if i >= w else []

    # removing edges if wanted
    edges = [(u, v) for u in range(n) for v in E[u] if u < v]

    m_0 = len(edges)
    m_wanted = m_0
    if delete_edges != 0:
        m_wanted = m_0 - delete_edges
    elif keep_edges_percent != 1:
        m_wanted = m_0 * keep_edges_percent

    random.shuffle(edges) # shuffling edges so that they are deleted randomly
    
    for u, v in edges:
        if m_0 <= m_wanted:
            break
            
        E[u].remove(v)
        E[v].remove(u)
        
        if Util.is_connected(E):
            m_0 -= 1
        else:
            E[u].append(v)
            E[v].append(u)

    amounts = [1 for _ in range(groups)]
    for i in range(agents-groups):
        r = math.floor(random.random()*groups)
        amounts[r] += 1
        
    selected = random.sample(range(n), 2 * agents)
    if agentsLeft:
        S = [buffer+w*(i+buffer) for i in range(0,agents)]
        D = [w*((i+buffer)+1) -1 -buffer for i in range(0,agents)]
    else:
        S = selected[0:agents]
        D = selected[agents:2*agents]

    S,D = partition_agents_to_groups(S,D,groups,keep_order)

    obstacle = [[] for _ in range(trucks)]
    # add moving obstacle if wanted
    for i in range(trucks):
        pos = -1
        while True:
            pos = random.randint(0,len(V)-1)
            if not any(pos in s for s in S) and not any(pos in d for d in D):
                break
        obstacle[i] += [(pos,0)]
        
        for t in range(1,1000):
            neighbors = [v for v in E[pos] if not any(v in s for s in S) and not any(v in d for d in D)]
            if len(neighbors) == 0:
                obstacle[i] += [(pos,t)]
                continue
            pos = neighbors[random.randint(0,len(neighbors)-1)]
            obstacle[i] += [(pos,t)]

            


    inst = MAPF_Inst(V,S,D,E,100,(w,h),obstacles_moving=obstacle)
    return inst