import Instances
import random
import Util
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import Visualization
import pandas as pd
import FB_solver


def reachability_analysis():
    inst = Instances.inst1
    V,S,D,E = inst.V,inst.S,inst.D,inst.E
    T = 2
    n = len(V)
    
    V_T = ["s", "d"] + [(v, t) for t in range(T + 1) for v in V]

    E_T = [[2 + v for s in S for v in s], []]
    
    for t in range(T):
        for v in range(n):
            neighbors = list(E[v]) + [v]
            E_T.append([2 + e + n * (t + 1) for e in neighbors])
            
    flat_D = [v for d in D for v in d]
    
    for v in range(n):
        if v in flat_D:
            E_T.append([1])
        else:
            E_T.append([])

    bg_V_T = list(V_T)
    bg_E_T = [list(edges) for edges in E_T]

    V_T, E_T = Util.remove_useless_vertices(V_T, E_T, True, True)
    
    Visualization.draw_time_expanded_network(V_T, E_T,bg_V_T,bg_E_T)

def generate_runtime_figure(alg, x, y, test_size, seed=None, log_scale=False,semi_log_scale = False, show_quadratic_ref=False,extended = False,buffer = 0,trucks = 0,type = "grid"):
    if x == "W":
        xlabel = "grid dimension w"
        W = [2,4,6,8,10,12,14,16,18,20,22]
        if extended: W = [5,10,15,20,25,30,40,50,60,80]
        H = [2]
        G = [1]
        X = W
    elif x == "H":
        xlabel = "number of agents p"
        W = [8]
        H = [1,2,3,4,5,6,7,8,9,10]
        G = [1]
        X = H
    elif x == "G":
        xlabel = "groups g"
        W = [5]
        H = [5]
        G = [1, 2, 3, 4, 5]
        X = G
        
    Runtimes, Timesteps,Averages, Failures = test(alg, W, H, G, test_size, seed=seed, type=type,buffer = buffer,trucks=trucks)
    
    if y == "R":
        ylabel = "runtime in seconds"
        Y = [Runtimes[i][j][k] for i in range(len(H)) for j in range(len(W)) for k in range(len(G))]
        
        if max(Y) * 1000 < 100:
            Y = [val * 1000 for val in Y]
            ylabel = "runtime in milliseconds"
            
    elif y == "T":
        ylabel = "timesteps"
        Y = [Timesteps[i][j][k] for i in range(len(H)) for j in range(len(W)) for k in range(len(G))]
    elif y == "F":
        ylabel = "failure rate"
        Y = [Failures[i][j][k] for i in range(len(H)) for j in range(len(W)) for k in range(len(G))]
        

    plt.figure(figsize=(8, 5))
    
    plt.plot(X, Y, marker='o', linestyle='-', color='b', label='Data')
    
    if log_scale:
        plt.xscale('log')
        plt.yscale('log')
        
        plt.gca().xaxis.set_major_formatter(ScalarFormatter())
        plt.gca().yaxis.set_major_formatter(ScalarFormatter())
        
        if show_quadratic_ref and len(Y) > 0 and Y[0] > 0:
            c = Y[0] / (X[0] ** 2)
            Y_ref = [c * (x_val ** 2) for x_val in X]
            plt.plot(X, Y_ref, marker='', linestyle=':', color='r', label='O(w²) reference')
            plt.legend(fontsize=12)
    elif semi_log_scale:
        plt.yscale('log')
        plt.gca().yaxis.set_major_formatter(ScalarFormatter())

    plt.xlabel(xlabel, fontsize=18)
    plt.ylabel(ylabel, fontsize=18)
    
    plt.xticks(X, fontsize=16)
    plt.yticks(fontsize=16)
    
    plt.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.show()

def compare_solvers(type,seed = None,test_size = 100,cars = True):
    if type == "game":
        R1,T1,A1,F1 = test("PR",[2],[5],[1],test_size,seed=seed, type = type,cars=cars)
        R2,T2,A2,F2 = test("FB",[2],[5],[1],test_size,seed=seed, type = type,cars=cars)
    if type == "delivery":
        R1,T1,A1,F1 = test("PR",[4],[10],[4],test_size,seed=seed, type = type)
        R2,T2,A2,F2 = test("FB",[4],[10],[4],test_size,seed=seed, type = type)

    data = {
        "Solver": ["PR", "FB"],
        "Runtime in s": R1[0][0]+R2[0][0],
        "Makespan": T1[0][0]+T2[0][0],
        "Average time": A1[0][0]+A2[0][0],
        "Failure rate": F1[0][0]+F2[0][0]
    }
    df = pd.DataFrame(data)
    print(df.to_latex(index=False, escape=False))

def observe_possible_dimension(type,seed = None,test_size = 100):
    if type == "game":
        R1,T1,A1,F1 = test("PR",[2],[5],[1],test_size,seed=seed, type = type)
        R2,T2,A2,F2 = test("FB",[2],[5],[1],test_size,seed=seed, type = type)
    if type == "delivery":
        R1,T1,A1,F1 = test("PR",[4],[10],[4],test_size,seed=seed, type = type)
        R2,T2,A2,F2 = test("FB",[4],[10],[4],test_size,seed=seed, type = type)

    data = {
        "Solver": ["PR", "FB"],
        "Runtime": R1[0][0]+R2[0][0],
        "Makespan": T1[0][0]+T2[0][0],
        "Average time": A1[0][0]+A2[0][0],
        "Failure rate": F1[0][0]+F2[0][0]
    }
    df = pd.DataFrame(data)
    print(df.to_latex(index=False, escape=False))

def test(alg,W,H,G,test_size,trucks = 0,seed = None, type = "horizontal",buffer = 0,cars = True):
    if seed != None: random.seed(seed)


    Runtimes = [[[0 for _ in G] for _ in W] for _ in H]
    Timesteps = [[[0 for _ in G] for _ in W] for _ in H]
    Averages = [[[0 for _ in G] for _ in W] for _ in H]
    Failures = [[[0 for _ in G] for _ in W] for _ in H]

    for h_index,h in enumerate(H):
        for w_index,w in enumerate(W):
            for g_index,g in enumerate(G):
                runtime = 0
                timesteps = 0
                average = 0
                failure = 0
                actual_size = test_size
                for _ in range(test_size):
                    seed_grid = random.randint(0, 100000) # makes a random seed for each instance, yielding the same instances for each algorithm as seed_grid is determined by seed
                    if type == "horizontal":
                        inst = Instances.generateGridHorizontalPaths(w,h,g,keep_edges_percent=0.9,trucks=trucks,seed=seed_grid,buffer = buffer)
                    elif type == "grid":
                        inst = Instances.generateGrid(w,w,h,g,trucks=trucks,keep_edges_percent=1)
                    elif type == "geometric":
                        inst = Instances.generateRandomGeometric(w,h,g,1,seed=seed_grid)
                    elif type == "game":
                        inst = Instances.generateGameInstance(w,h,seed=seed_grid,cars=cars)
                    elif type == "delivery":
                        inst = Instances.generateDeliveryInstance(w,w-2,h,g,seed=seed_grid)
                    valid,R,P = Util.run_solver(inst,alg,printing=False)
                    if valid == False:
                        actual_size -= 1
                        failure += 1
                        continue
                    runtime += R
                    timesteps += len(P[0])-1
                    average += sum(next(i for i,v in enumerate(p) if v == p[-1])/len(P) for p in P)

                runtime /= actual_size
                timesteps /= actual_size
                average /= actual_size
                failure /= test_size

                Runtimes[h_index][w_index][g_index] = runtime
                Timesteps[h_index][w_index][g_index] = timesteps
                Averages[h_index][w_index][g_index] = average
                Failures[h_index][w_index][g_index] = failure

    print(Runtimes)
    print(Timesteps)
    print(Averages)
    print(Failures)

    return Runtimes,Timesteps,Averages,Failures
