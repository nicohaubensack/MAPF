from scipy.optimize import linear_sum_assignment 

def pr_solver(V, S, D, E, obstacles):

    g = len(S)
    agent_group = [i for i in range(g) for _ in range(len(S[i]))]
    Paths = []

    for i in range(g):
        Distances = getDistances(S[i], D[i], E)

        _, solution = distribute_destinations(Distances)

        for j in range(len(S[i])):
            
            Paths += [collision_reduced_path_search(Paths,S[i][j], D[i][solution[j]], E)]

    timestep = 0
    safety_limit = 10000
    iteration = 0

    blocked_vertices,blocked_edges = obstacles[0],obstacles[1]

    dodge_memory = [[-1 for _ in range(len(Paths))] for _ in range(len(Paths))]
    dodged_at_timestep = []

    while True:
        if iteration >= safety_limit: # to avoid deadloops (solver not complete)
            return finalized_P(V,Paths)
            
        max_l = max([len(p) for p in Paths])
        if timestep >= max_l:
            break

        collision_resolved = False
        # Check all pairs for collisions at the current timestep
        for i1 in range(len(Paths)):

            # obstacle collision checks
            if len(Paths[i1]) > timestep:
                pos = Paths[i1][timestep]
                if len(Paths[i1]) > timestep + 1:

                    next_pos = Paths[i1][timestep+1]
                    if (pos,next_pos,timestep) in blocked_edges: # delaying agent
                        Paths[i1].insert(timestep, pos)
                        collision_resolved = True
                        break

                if collision_resolved: break

                
                if (pos,timestep) in blocked_vertices:
                    
                    timestep = dodgeObstacle(i1,Paths,E,blocked_vertices,blocked_edges,pos,dodge_memory,dodged_at_timestep,timestep)

                    if timestep is False: return finalized_P(V,Paths)
                    collision_resolved = True
                    
            
            if collision_resolved: break

            # agent collision checks
            for i2 in range(i1 + 1, len(Paths) ):

                p1, p2 = Paths[i1], Paths[i2]
                l1, l2 = len(p1), len(p2)
                
                pos1 = p1[timestep] if timestep < l1 else p1[-1]
                pos1_t2 = p1[timestep + 1] if timestep + 1 < l1 else None
                pos2 = p2[timestep] if timestep < l2 else p2[-1]
                pos2_t2 = p2[timestep + 1] if timestep + 1 < l2 else None

                # Edge collision
                if pos1_t2 is not None and pos2_t2 is not None and pos1 == pos2_t2 and pos2 == pos1_t2: 
                    
                    if agent_group[i1] == agent_group[i2] and i1 not in dodged_at_timestep and i2 not in dodged_at_timestep: # agents are in the same group, we can switch their paths
                        Paths[i1] = p1[:timestep] + p2[timestep+1:]
                        Paths[i2] = p2[:timestep] + p1[timestep+1:]
                    else: # agents are in a different group, one needs to step away
                        dodgeGroupCollision(E,blocked_vertices,blocked_edges,dodge_memory,dodged_at_timestep,Paths,timestep,i1,i2)

                    collision_resolved = True
                    break

                # Vertex collision
                if pos1 == pos2: 
                    i_shorter = i1 if l1 <= l2 else i2
                    i_longer = i2 if l1 <= l2 else i1
                    
                    
                    if timestep >= len(Paths[i_shorter]): # One agent is resting at its destination

                        if agent_group[i1] == agent_group[i2]: # agents are in the same group
                            # Pad the shorter path up to the current timestep
                            pad_len = timestep - len(Paths[i_shorter])
                            if pad_len > 0:
                                Paths[i_shorter] += [Paths[i_shorter][-1]] * pad_len
                            
                            # Swap remaining paths
                            remaining_p_longer = Paths[i_longer][timestep + 1:]
                            Paths[i_shorter] += remaining_p_longer
                            Paths[i_longer] = Paths[i_longer][:timestep + 1]
                        else: # agents are in a different group, now we need to backtrack path
                            timestep = backtrackPath(i_shorter,i_longer,Paths,E,D,timestep,dodged_at_timestep)
                            if timestep is False:
                                return finalized_P(V,Paths)
                            
                    else: # Both agents are still actively moving
                        if Paths[i1][timestep - 1] == Paths[i1][timestep]:
                            Paths[i2].insert(timestep, Paths[i2][timestep - 1])
                        elif Paths[i2][timestep - 1] == Paths[i2][timestep]:
                            Paths[i1].insert(timestep, Paths[i1][timestep - 1])
                        elif dodge_memory[i1][i2] != -1:
                            Paths[dodge_memory[i1][i2]].insert(timestep, Paths[dodge_memory[i1][i2]][timestep - 1])
                        else:
                            Paths[i_shorter].insert(timestep, Paths[i_shorter][timestep - 1])
                    
                    collision_resolved = True
                    break

            if collision_resolved: break # Break outer loop to restart checking the same timestep

        iteration += 1   
        # Only advance to the next timestep if the current one is completely collision-free
        if not collision_resolved:

            # checking for a cycle of agents
            if check_for_loops(g,agent_group,Paths,timestep): continue
                
            # advancing to next time step
            timestep += 1
            dodged_at_timestep = []
    
    return finalized_P(V,Paths)

def dodgeGroupCollision(E,blocked_vertices,blocked_edges,dodge_memory,dodged_at_timestep,Paths,timestep,i1,i2):
    best_rating = -1
    i_chosen = 0
    choice_final = 0
    waiting = []
    for i in [i1,i2]:
        i_other = i1 if i == i2 else i2
        pos = Paths[i][timestep]
        filtered = [neighbor for neighbor in E[pos] if (neighbor,timestep+1) not in blocked_vertices and (pos,neighbor,timestep) not in blocked_edges and neighbor != Paths[i_other][timestep]]
        if len(filtered) == 0:
            continue
        
        choice_rating = 0

        agents_blocked_vertices = [path[timestep+1] if len(path) > timestep + 1 else path[-1] for path in Paths]
        agents_waiting = [path[timestep] == path[timestep+1] if len(path) > timestep + 1 else True for path in Paths]
        agents_blocked_edges = [(path[timestep],path[timestep+1]) for path in Paths if len(path) > timestep + 1]
        for neighbor in filtered:
            if neighbor not in agents_blocked_vertices and (neighbor,pos) not in agents_blocked_edges and (neighbor,pos,timestep) not in blocked_edges and (pos,timestep+1) not in blocked_vertices:
                
                choice_rating = 2
                choice = neighbor
                break
            elif not any(agents_blocked_vertices[idx] == neighbor and agents_waiting[idx] == True for idx in range(len(Paths))):
                choice_rating = 1
                choice = neighbor

        if choice_rating == 0: choice = filtered[0] 
        
        if choice_rating > best_rating or (choice_rating == best_rating and dodge_memory[i1][i2] == i):
            best_rating = choice_rating
            i_chosen = i
            choice_final = choice
            waiting = [idx for idx in range(len(Paths)) if agents_blocked_vertices[idx] == choice and agents_waiting[idx] == True]
            
    
    if best_rating == -1: # both agents wait for the environment to clear up 
        Paths[i2].insert(timestep, Paths[i2][timestep])
        Paths[i1].insert(timestep, Paths[i1][timestep])
        return

    path_after = collision_reduced_path_search([p[timestep+1:] for idx,p in enumerate(Paths) if idx != i_chosen],choice_final, Paths[i_chosen][-1], E)
    Paths[i_chosen] = Paths[i_chosen][:timestep+1] + path_after
    agents_to_dodge = [agent for agent in range(len(Paths)) if agent != i_chosen and agents_blocked_vertices[agent] == choice_final or (len(Paths[agent]) > timestep +1 and Paths[agent][timestep+1] == Paths[i_chosen][timestep] and Paths[agent][timestep] == Paths[i_chosen][timestep+1])]
    for agent in agents_to_dodge:
        dodge_memory[min(i_chosen,agent)][max(i_chosen,agent)] = agent
    for agent in waiting:
        Paths[agent].insert(timestep+1,Paths[i_chosen][timestep])
    dodge_memory[i1][i2] = i_chosen
    dodged_at_timestep += [i_chosen]

def backtrackPath(i_shorter,i_longer,Paths,E,D,timestep,dodged_at_timestep):
    collision_resolved = False
    for j in range(len(Paths[i_shorter]) - 1, -1, -1):
        val_at_j = Paths[i_shorter][j]
        for neighbor in E[val_at_j]:
            if all(Paths[i_longer][t] != neighbor for t in range(j-1, len(Paths[i_longer]))) and all(neighbor not in d for d in D):
                wait_until = j
                for t in range(len(Paths[i_longer]) - 1, j-1, -1):
                    if Paths[i_longer][t] == val_at_j:
                        wait_until = t
                        break
                
                distribution = []
                # reset all paths after timestep j
                for idx in range(len(Paths)):
                    distribution += [Paths[idx][-1]]
                    if len(Paths[idx]) > j + 1:
                        Paths[idx] = Paths[idx][:j + 2]

                for t in range(j+1, wait_until+1):
                    Paths[i_shorter].insert(t,neighbor)
                Paths[i_shorter].insert(wait_until+1,val_at_j)

                for idx in range(len(Paths)):
                    path_after = collision_reduced_path_search([p[j+1:] for p in Paths],Paths[idx][-1], distribution[idx], E)
                    Paths[idx] = Paths[idx][:-1] + path_after

                timestep = j+1
                collision_resolved = True
                dodged_at_timestep.clear()
                break
            
        if collision_resolved:break
    if not collision_resolved: # cannot backtrack, as no position possible
        return False
    return timestep

def dodgeObstacle(i1,Paths,E,blocked_vertices,blocked_edges,pos,dodge_memory,dodged_at_timestep,timestep):
    
    prev_pos = Paths[i1][timestep-1]
    possible_blocked_vertices = E[prev_pos] + [prev_pos]
    filtered = [neighbor for neighbor in possible_blocked_vertices if (neighbor,timestep) not in blocked_vertices and (prev_pos,neighbor,timestep-1) not in blocked_edges]
    if len(filtered) == 0: 
        return False

    choice_rating = 0

    agents_blocked_vertices = [path[timestep] if len(path) > timestep  else path[-1] for path in Paths]
    agents_waiting = [path[timestep] == path[timestep-1] if len(path) > timestep else True for path in Paths]
    agents_blocked_edges = [(path[timestep-1],path[timestep]) for path in Paths if len(path) > timestep]
    for neighbor in filtered:
        if neighbor not in agents_blocked_vertices and (neighbor,pos) not in agents_blocked_edges:
            choice_rating = 2
            choice = neighbor
            break
        elif not any(agents_blocked_vertices[i] == neighbor and agents_waiting[i] == True for i in range(len(Paths))):
            choice_rating = 1
            choice = neighbor

    if choice_rating == 0: 
        choice = filtered[0]
    waiting = [idx for idx in range(len(Paths)) if agents_blocked_vertices[idx] == choice and agents_waiting[idx] == True]
    for a in waiting:
        Paths[a].insert(timestep,prev_pos) # Force agent to move to trigger edge collision

    path_after = collision_reduced_path_search([p[timestep:] for idx,p in enumerate(Paths) if idx != i1],choice, Paths[i1][-1], E)
    Paths[i1] = Paths[i1][:timestep] + path_after
    

    agents_to_dodge = [agent for agent in range(len(Paths)) if agent != i1 and agents_blocked_vertices[agent] == choice or (len(Paths[agent]) > timestep +1 and Paths[agent][timestep] == Paths[i1][timestep-1] and Paths[agent][timestep-1] == Paths[i1][timestep])]
    for agent in agents_to_dodge:
        dodge_memory[min(i1,agent)][max(i1,agent)] = agent

    if len([agent for agent in range(len(Paths)) if (len(Paths[agent]) > timestep +1 and Paths[agent][timestep] == Paths[i1][timestep-1] and Paths[agent][timestep-1] == Paths[i1][timestep])]) > 0: 
        timestep = timestep -1
    dodged_at_timestep += [i1]

    return timestep

def check_for_loops(groups,agent_group,Paths,timestep):
    for g in range(groups):
        agents_to_check = [i for i in range(len(agent_group)) if agent_group[i] == g and len(Paths[i]) > timestep+1] # all agents in group g of length greater than t+1
        for a in agents_to_check:
            cycle = [a]
            found = False
            while True:
                v = Paths[a][timestep+1]
                if v == Paths[a][timestep]: break
                new_agent = False
                for a2 in [temp for temp in agents_to_check if temp != a]:
                    if Paths[a2][timestep] == v and Paths[a2][timestep+1] != v:
                        for i,a3 in enumerate(cycle):
                            if a3 != a and Paths[a3][timestep] == Paths[a2][timestep+1]:
                                cycle = cycle[i:] + [a2]
                                found = True
                                break
                        if found: break
                        cycle += [a2]
                        a = a2
                        new_agent = True
                        break
                if found: break
                if not new_agent: break
            
            if found: 
                after_paths = [p[timestep+2:] for p in Paths]
                for i,a in enumerate(cycle):
                    if i == 0:continue
                    Paths[a] = Paths[a][:timestep+1] + after_paths[cycle[i-1]]
                Paths[cycle[0]] = Paths[cycle[0]][:timestep+1] + after_paths[cycle[-1]]

                return True
            
    return False

def finalized_P(V,Paths):
    P = [[V[v] for v in p] for p in Paths]
    l = max([len(p) for p in P])
    P = [p + [p[-1]] * (l - len(p)) for p in P]

    return P

def distribute_destinations(Distances):
    # Gather unique valid distances
    unique_dists = []
    for row in Distances:
        for d in row:
            if d != -1 and d not in unique_dists:
                unique_dists.append(d)
    unique_dists.sort()
    
    if not unique_dists:
        return -1, []
    
    n = len(Distances)
    
    low = 0
    high = len(unique_dists) - 1
    value = -1
    
    # Binary search for the optimal bottleneck value
    while low <= high:
        mid = (low + high) // 2
        limit = unique_dists[mid]
        
        # Create a temporary cost matrix
        cost_matrix = [[0 for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                if Distances[i][j] != -1 and Distances[i][j] <= limit:
                    cost_matrix[i][j] = 0
                else:
                    cost_matrix[i][j] = 1
        
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        # Check if the assignment is valid
        valid = True
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r][c] >= 1:
                valid = False
                break
        
        if valid:
            value = limit
            high = mid - 1
        else:
            low = mid + 1
            
    if value == -1:
        return -1, []
    
    # Refine the assignment to minimize the sum of distances under the bottleneck limit
    final_cost_matrix = [[1000000 for _ in range(n)] for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if Distances[i][j] != -1 and Distances[i][j] <= value:
                final_cost_matrix[i][j] = Distances[i][j]
                
    row_ind, col_ind = linear_sum_assignment(final_cost_matrix)
    solution = [int(x) for x in col_ind]
    
    return value, solution

def collision_reduced_path_search(P_current, s, d, E):
    if s == d:
        return [s]

    already = {v for p in P_current for v in p}

    queue = [(0, [s])]
    reached = set()

    while queue:
        cost, path = queue.pop(0)
        v = path[-1]

        if v == d:
            return path
        if v in reached:
            continue

        reached.add(v)

        for w in E[v]:
            if w not in reached:
                extra = 1 / (len(already) + 1) if w in already else 0
                queue.append((cost + 1 + extra, path + [w]))

        queue.sort(key=lambda x: x[0])

    return []      
    
def getDistances(S, D, E):
    Distances = [[-1 for _ in D] for _ in S]
    
    dest_map = {d: i for i, d in enumerate(D)}
    num_dests = len(D)
    
    for idx, s in enumerate(S):
        queue = [(s, 0)]
        visited = {s}
        found_count = 0
        
        while queue:
            v, t = queue.pop(0)
            
            if v in dest_map:
                idx2 = dest_map[v]
                if Distances[idx][idx2] == -1:
                    Distances[idx][idx2] = t
                    found_count += 1
                    if found_count == num_dests:
                        break
            
            for v_ in E[v]:
                if v_ not in visited:
                    visited.add(v_)
                    queue.append((v_, t + 1))
                    
    return Distances

