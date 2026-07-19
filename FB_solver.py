import Util
import time
from scipy.optimize import linprog
from scipy.sparse import coo_matrix
import PR_solver

def fb_solver(V,S,D,E,obstacles):
    P_PR = PR_solver.pr_solver(V,S,D,E,obstacles)
    valid = Util.validate_solution(V,E,S,D,P_PR,obstacles,False)

    # determine if upper bound is feasible
    iteratively = False
    if valid == False:  
        upper_bound = 10000
        iteratively = True
    else: upper_bound = len(P_PR[0])

    lower_bound = max([get_shortest_makespan(S[i], D[i], E) for i in range(len(S))])

    if lower_bound >= upper_bound:
        return P_PR

    solution = False
    low = lower_bound
    high = upper_bound - 1

    while low <= high:
        if iteratively:
            T = low
        else:
            T = (low + high) // 2

        V_T,E_T = Util.create_time_expanded_network(T,V,S,D,E,obstacles,True,True)
        if E_T == False:
            P = False
        else:
            P = runSimplex(V,V_T,E_T,S,D,T)
        if P == False:
            if iteratively:
                low += 1
            else:
                low = T+1
        else:
            if iteratively:
                solution = extract_minimal_paths(V,P)
                break
            else:
                P_pure = extract_minimal_paths(V,P)
                last_time_not_at_dest = max((i for p in P_pure for i in range(len(p)) if p[i] != p[-1]),default=-1)
                P_pure = [p[:last_time_not_at_dest+2] for p in P_pure]

                T = last_time_not_at_dest +1
                solution = P_pure
                high = T - 1

    if solution == False: return False        
    return solution



def runSimplex(V, V_T, E_T, S, D,T):
    n = len(V_T)
    agents = sum(len(s) for s in S)
    groups = len(S)
    
    rev_E = getReversedE(E_T)

    if len(rev_E[1]) < agents or len(E_T[0]) < agents : return False
    E_list = []
    for v in range(n):
        E_list += [(v, e) for e in E_T[v]]
    
    m = len(E_list)

    c = [0 for _ in range(groups) for _ in range(m)]
    
    rows, cols, vals = [], [], []

    # creating a sparse matrix for increased efficiency
    for g in range(groups):
        for v in range(2, n): 
            # coordinates of constraint rows
            idx1 = g*2*(n-2)+2*(v-2)
            idx2 = g*2*(n-2)+2*(v-2)+1

            for index, (u, w) in enumerate(E_list):
                col_idx = index + m * g
                if w == v:
                    rows.extend([idx1, idx2])
                    cols.extend([col_idx, col_idx])
                    vals.extend([1, -1])
                elif u == v:
                    rows.extend([idx1, idx2])
                    cols.extend([col_idx, col_idx])
                    vals.extend([-1, 1])
    
    b = [0 for _ in range(2 * (n - 2) * groups)]
    
    # Capacity constraints
    base_idx = 2 * (n - 2) * groups

    for index in range(m):
        row_idx = base_idx + index
        for g in range(groups):
            rows.append(row_idx)
            cols.append(index + m * g)
            vals.append(1)
        b.append(1)
        
    A = coo_matrix((vals, (rows, cols)), shape=(base_idx + m, m * groups))
    
    x_bounds = [(0, 1) for _ in range(groups) for _ in range(m)]

    e_to_idx = {v: i for i, v in enumerate(E_list)}
    
    v_to_idx = {v:i for i,v in enumerate(V)}
    

    for i in range(agents):
        (v,_) = V_T[E_T[0][i]]
        s_idx = v_to_idx[v]
        for g in range(groups):
            if s_idx in S[g]:
                # this contributes to the value of the flow
                c[g * m + e_to_idx[(0, E_T[0][i])]] = -1
            else:
                # restrict other commodities on start vertex
                x_bounds[g * m + e_to_idx[(0, E_T[0][i])]] = (0, 0)


    for i in range(len(rev_E[1])):
        (v,_) = V_T[rev_E[rev_E[1][i]][0]]
        d_idx = v_to_idx[v]
        for g in range(groups):
            if d_idx not in D[g]:
                # restrict other commodities on destination vertex
                x_bounds[g * m + e_to_idx[(rev_E[1][i], 1)]] = (0, 0)


    max_t_to_award = min(7,T-1) # cap at 7 to avoid numeric problems and higher runtime
    max_bonus = 0.9
    # award edges that remain an agent on a destination vertex
    for t_lower in range(0,max_t_to_award):
        t = T-1-t_lower
        bonus = max_bonus / (agents*max_t_to_award)
        for i in range(m):
            (e1,e2) = E_list[i]
            v1,v2 = V_T[e1],V_T[e2]
            if len(v1)< 2 or len(v2)< 2: continue
            (v1_node,t1) = v1
            (v2_node,t2) = v2

            if isinstance(v1_node, str) and v1_node.endswith('_'):
                clean_v1 = v1_node.rstrip('_')

                if clean_v1 == str(v2_node) and t2 == t1 + 1 and t1 == t:
                    physical_vertex = v2_node 
                    
                    if physical_vertex in v_to_idx:
                        v_idx = v_to_idx[physical_vertex]
                        
                        for g in range(groups):
                            if v_idx in D[g]:
                                c[g * m + i] = -bonus

    
    if groups > 1: integrality = [1 for _ in range(groups * m)]
    else: integrality = []
    
    res = linprog(
        c,
        A_ub=A,
        b_ub=b,
        bounds=x_bounds,
        integrality=integrality,
        method='highs'
    )
    
    if not res.success: return False
    
    coeff = [int(round(x)) for x in res.x]

    # Construct paths from LP
    Paths = [[] for _ in E_T[0]]
    E_list_filtered = [E_list[i] for i in range(m) if any(coeff[i + m * g] == 1 for g in range(groups))]

    for (idx, v) in enumerate(E_T[0]):
        queue = v 
        reached = []
        while True:
            status = 0
            reached += [V_T[queue]]
            for (u, w) in E_list_filtered:
                if u == queue:
                    if w == 1:
                        status = 2
                        break
                    status = 1
                    queue = w
                    break
            if status == 0:  # no flow from that agent
                return False
            elif status == 2:  # reached t node
                Paths[idx] = (reached)
                break
    return Paths

def getReversedE(E):
    n = len(E)
    rev_E = [[] for _ in range(n)]
    for u in range(n):
        for v in E[u]:
            rev_E[v].append(u)

    return rev_E

def extract_minimal_paths(V,P):
    for idx,p in enumerate(P):
        P[idx] = [v for (v,t) in p if v in V]

    return P
 
def get_shortest_makespan(S,D,E):
    value,_ = PR_solver.distribute_destinations(PR_solver.getDistances(S,D,E))
    return value