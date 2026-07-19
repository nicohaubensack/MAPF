import FB_solver
import PR_solver
import time
import Visualization

def create_time_expanded_network(T,V_0,S,D,E_0,obstacles,delete_in,delete_out,for_visualization = False):
    n = len(V_0)
    V = ["s","d"]
    V = V + [(v,t) for t in range(T+1) for v in V_0]
    V = V + [(f"{v}_",t) for t in range(T+1) for v in V_0]

    E = [[2+v for s in S for v in s],[]]
    for t in range(T+1):
        for v in range(n):
            E.append([v_index(v,t,True,n,T)])

    for t in range(T):
        for v in range(n):
            #last time step not included because different logic
            E.append([])
            for neighbor in E_0[v]+[v]:
                if (neighbor,t+1) not in obstacles[0] and (v,neighbor,t) not in obstacles[1] and (neighbor,v,t) not in obstacles[1]: # filtering out obstacles
                    E[-1] += [v_index(neighbor,t+1,False,n,T)] 

    flat_D = [v for d in D for v in d]
    for v in range(n):
        E.append([1]) if v in flat_D else E.append([])

    if delete_in or delete_out: V,E = remove_useless_vertices(V,E,delete_in,delete_out)
    
    if for_visualization == False:
        for t in range(T):
            V_t = [(v,time) for (v,time) in V[2:] if time == t and "_" not in v]
            if(len(V_t) < sum(len(s) for s in S)):
                return False,False
            

    V,E = intersection_gadgets(V,E)
    
    return V,E

def v_index(v,t,out,n,T):
    if not out:
        return 2+n*t + v
    else:
        return 2+n*(T+1) + n*t + v
    
def remove_useless_vertices(V_T, E_T,del_in,del_out):
    n_nodes = len(V_T)
    
    # reachable from s
    reachable_from_s = set([0])
    queue = [0]
    while queue:
        curr = queue.pop(0)
        for neighbor in E_T[curr]:
            if neighbor not in reachable_from_s:
                reachable_from_s.add(neighbor)
                queue.append(neighbor)

    # Reverse edges
    rev_E = FB_solver.getReversedE(E_T)

    # can reach t
    can_reach_t = set([1])
    queue = [1]
    while queue:
        curr = queue.pop(0)
        for neighbor in rev_E[curr]:
            if neighbor not in can_reach_t:
                can_reach_t.add(neighbor)
                queue.append(neighbor)

    # Find useful nodes and sort them to maintain order
    if del_in and del_out:useful_nodes = reachable_from_s.intersection(can_reach_t)
    elif del_in: useful_nodes = reachable_from_s
    else: useful_nodes = can_reach_t
    useful_nodes.update([0, 1])  # Ensure s and t are always kept
    useful_nodes = sorted(list(useful_nodes))

    # Create a dictionary to map old indices to new indices
    new_index = {old_id: new_id for new_id, old_id in enumerate(useful_nodes)}

    new_V = [V_T[i] for i in useful_nodes]
    new_E = [[new_index[n] for n in E_T[i] if n in new_index] for i in useful_nodes]

    return new_V, new_E

def get_obstacle_nodes(obstacles_static, obstacles_moving):
    vertices = list(obstacles_static[0])
    edges = list(obstacles_static[1])
    for path in obstacles_moving:
        for i in range(len(path)):
            (v,t) = path[i]
            
            vertices.append((v, t))
            if i < len(path) - 1:
                (u,_) = path[i+1]
                edges.append((v, u, t))
                edges.append((u, v, t))

    return [vertices, edges]

def intersection_gadgets(V_T,E_T):
    queue = []
    n = int((len(V_T) - 2) / 2)
    for i_pure in range(n):
        i = 2+n+i_pure
        (v_in,t) = V_T[i-n]
        j = -1
        for it,u_it in enumerate(V_T):
            if it <2:continue
            if u_it == (v_in,t+1):
                j = it
                break
        if j == -1: continue

        for e in E_T[i]:
            if e < 2: continue
            (w,t_1) = V_T[e]
            j2 = -1
            for it,u_it in enumerate(V_T):
                if it <2:continue
                if u_it == (w,t_1 - 1):
                    j2 = it
                    break
                
            if j2 == -1: continue
            if i == j2 + n: continue

            if j in E_T[j2 + n]:
                (_,t) = V_T[i]

                is_duplicate = False
                for (q_out1, q_out2, q_in1, q_in2, q_t) in queue:
                    # Check if the reversed pair (j2+n, i) at time t is already in the queue
                    if q_out1 == j2 + n and q_out2 == i and q_t == t:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    queue += [(i, j2 + n, e, j, t)]

    for (out1, out2, in1, in2, t) in queue:
        E_T[out1].remove(in1)
        E_T[out2].remove(in2)
        
        g1 = len(V_T)
        g2 = len(V_T) + 1
        
        E_T[out1] += [g1]
        E_T[out2] += [g1]
        
        V_T += [(f"{out1},{out2}_1", t)]
        V_T += [(f"{out1},{out2}_2", t)]
        
        E_T += [[g2]]
        E_T += [[in1, in2]]

    return V_T,E_T

def is_connected(E):
    # note that this only works on undirected graphs
    m = len(E)
    visited = {0}
    queue = [0]
    while queue:
        curr = queue.pop(0)
        for neighbor in E[curr]:
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
    return len(visited) == m

def solve_instance(algs,inst):
    Runtimes = []
    Timesteps = []
    output = []
    for i,alg in enumerate(algs):
        valid,Runtime,P = run_solver(inst,alg)

        if valid == False:
            print(f"no valid solution found using {alg}")
        if P == False:
            continue
        T = len(P[0])-1
        print(f"{alg} runtime: {Runtime:.4f}s, timesteps: {T}")
        Runtimes += [Runtime]
        Timesteps += [T]
        output += [(alg,P)]
    
    Visualization.draw_grid_mapf_simulation(inst,directed=True, P=output)
    return output

def run_solver(inst,alg,printing = True):
    t0 = time.time()
    V,S,D,E,CellSize,GridDimensions,imagePath,obstacles = inst.V,inst.S,inst.D,inst.E,inst.CellSize,inst.GridDimension,inst.imagePath,inst.obstacles
    if len(V) < [len(s) for s in S][0]: 
        print("Error: Not enough vertices for the number of agents.")
        return False
    obstacles_copy = [list(obstacles[0]), list(obstacles[1])]
    
    if alg == "PR":
        P = PR_solver.pr_solver(V, S, D, E,obstacles_copy)
    elif alg == "FB":
        P = FB_solver.fb_solver(V, S, D, E,obstacles_copy)
    valid = True
    if (validate_solution(V,E,S,D,P,obstacles,printing) == False): 
        valid = False
    Runtime = time.time() - t0
    return valid,Runtime,P

def validate_solution(V, E, S, D, P, obstacles,printing):

    if P == False: 
        if(printing): ("Solution invalid, algorithm terminated")
        return False
    T = len(P[0])

    group_of_p = [i for i in range(len(S)) for _ in range(len(S[i]))]

    if len(group_of_p) > len(P): print("Solution invalid, not all agents were routed")
        
    v_to_idx = {v: i for i, v in enumerate(V)}
    
    for i, p in enumerate(P):

        P_without_p = [P[j] for j in range(len(P)) if j != i]
        
        for t in range(T):
            if p[t] not in V: 
                if(printing): print(f"Solution invalid, agent {i} uses invalid vertex: {p[t]}")
                return False
                
            idx_t = v_to_idx[p[t]]
            
            if any(p2[t] == p[t] for p2 in P_without_p):
                if(printing): print(f"Solution invalid, agents collide on vertex {p[t]} at t={t}")
                return False
                
            if (idx_t, t) in obstacles[0]: 
                if(printing): print(f"Solution invalid, agent {i} uses blocked vertex {p[t]} at t={t}")
                return False

            if t < T - 1 and p[t] != p[t+1]:
                idx_t1 = v_to_idx[p[t+1]]
                
                if idx_t1 not in E[idx_t]: 
                    if(printing): print(f"Solution invalid, agent {i} uses non-existent edge {p[t]} -> {p[t+1]}")
                    return False
                    
                if any((p2[t], p2[t+1]) == (p[t+1], p[t]) for p2 in P_without_p): 
                    if(printing): print(f"Solution invalid, agents swap-collide on edge {p[t]}-{p[t+1]} at t={t}")
                    return False
                    
                if (idx_t, idx_t1, t) in obstacles[1]:
                    if(printing): print(f"Solution invalid, agent {i} uses blocked edge {p[t]}-{p[t+1]} at t={t}")
                    return False
                
        if v_to_idx[p[-1]] not in D[group_of_p[i]]:
            if(printing): print(f"Solution invalid, agent {i} not on a correct destination vertex. (Ended at {p[-1]})")
            return False
            
        if v_to_idx[p[0]] not in S[group_of_p[i]]:
            if(printing): print(f"Solution invalid, agent {i} not on a correct start vertex. (Started at {p[0]})")
            return False

    if(printing): print("Solution is valid!")
    return True